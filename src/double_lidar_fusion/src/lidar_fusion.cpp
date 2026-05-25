#include "double_lidar_fusion/lidar_fusion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

using sensor_msgs::msg::LaserScan;

LidarFusion::LidarFusion(const rclcpp::NodeOptions & options)
: Node("double_lidar_fusion", options)
{
  declare_parameter("scan1_topic",          "/scan1");
  declare_parameter("scan2_topic",          "/scan2");
  declare_parameter("fused_topic",          "scan");
  declare_parameter("frame_id",             "laser");
  declare_parameter("lidar1_angle_deg",      45.0);
  declare_parameter("lidar1_x_offset_m",      0.3);
  declare_parameter("lidar1_y_offset_m",      0.235);
  declare_parameter("lidar2_angle_deg",     -135.0);
  declare_parameter("lidar2_x_offset_m",     -0.3);
  declare_parameter("lidar2_y_offset_m",     -0.235);
  declare_parameter("scan_timeout_sec",       0.5);
  declare_parameter("publish_rate_hz",       12.0);

  get_parameter("scan1_topic",          scan1_topic_);
  get_parameter("scan2_topic",          scan2_topic_);
  get_parameter("fused_topic",          fused_topic_);
  get_parameter("frame_id",             frame_id_);
  get_parameter("lidar1_angle_deg",     lidar1_angle_deg_);
  get_parameter("lidar1_x_offset_m",    lidar1_x_offset_m_);
  get_parameter("lidar1_y_offset_m",    lidar1_y_offset_m_);
  get_parameter("lidar2_angle_deg",     lidar2_angle_deg_);
  get_parameter("lidar2_x_offset_m",    lidar2_x_offset_m_);
  get_parameter("lidar2_y_offset_m",    lidar2_y_offset_m_);
  get_parameter("scan_timeout_sec",     scan_timeout_sec_);
  get_parameter("publish_rate_hz",      publish_rate_hz_);

  // 先用节点时钟初始化时间戳，保证后续做时间差时时间源一致。
  last_scan1_time_ = this->now();
  last_scan2_time_ = this->now();

  // 用 best-effort QoS 订阅：2D 雷达驱动（如 lslidar）通常按 sensor data
  // 发布，best-effort 订阅同时兼容 reliable / best-effort 发布者，避免因
  // QoS 不匹配收不到数据。
  auto qos = rclcpp::SensorDataQoS();
  scan1_sub_ = create_subscription<LaserScan>(
      scan1_topic_, qos,
      std::bind(&LidarFusion::scan1Cb, this, std::placeholders::_1));
  scan2_sub_ = create_subscription<LaserScan>(
      scan2_topic_, qos,
      std::bind(&LidarFusion::scan2Cb, this, std::placeholders::_1));

  fused_scan_pub_ = create_publisher<LaserScan>(fused_topic_, 10);

  if (publish_rate_hz_ <= 0.0) {
    publish_rate_hz_ = 12.0;
  }
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LidarFusion::publishFused, this));

  RCLCPP_INFO(get_logger(),
      "double_lidar_fusion 启动: %s + %s -> %s (frame=%s), 超时 %.2fs, %.1fHz; "
      "单台掉线时自动降级为只输出存活的一台。",
      scan1_topic_.c_str(), scan2_topic_.c_str(), fused_topic_.c_str(),
      frame_id_.c_str(), scan_timeout_sec_, publish_rate_hz_);
}

/* ============================================================ */
void LidarFusion::cacheGeometry(const LaserScan::ConstSharedPtr & scan)
{
  if (angle_increment_rad_ == 0.0 && scan->angle_increment > 0.0) {
    angle_increment_rad_ = scan->angle_increment;
    bucket_count_ = static_cast<size_t>(std::round(2 * M_PI / angle_increment_rad_));
  }
}

void LidarFusion::scan1Cb(const LaserScan::ConstSharedPtr msg)
{
  last_scan1_ = msg;
  last_scan1_time_ = this->now();
  cacheGeometry(msg);
}

void LidarFusion::scan2Cb(const LaserScan::ConstSharedPtr msg)
{
  last_scan2_ = msg;
  last_scan2_time_ = this->now();
  cacheGeometry(msg);
}

/* ============================================================ */
void LidarFusion::integrateOneLidar(const LaserScan::ConstSharedPtr & scan,
                                    double cos_theta, double sin_theta,
                                    double x_offset, double y_offset,
                                    LaserScan & fused_scan) const
{
  constexpr double ANGLE_MIN = -M_PI;          // 输出激光从 -π 开始
  constexpr float  MERGE_THRESHOLD_M = 0.03f;  // 3 cm 死区，抑制闪烁
  constexpr float  MIN_VALID_RANGE   = 0.05f;  // 过滤近场噪声与 0 值
  const size_t     TOTAL_BUCKETS     = bucket_count_;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const float raw_range = scan->ranges[i];
    if (!std::isfinite(raw_range)) continue;
    if (raw_range < MIN_VALID_RANGE) continue;
    if (raw_range < scan->range_min || raw_range > scan->range_max) continue;
    const double local_angle = scan->angle_min + scan->angle_increment * static_cast<double>(i);
    const double local_x     = raw_range * std::cos(local_angle);
    const double local_y     = raw_range * std::sin(local_angle);
    /* --- 局部坐标 -> 机器人基坐标 --- */
    const double base_x = local_x * cos_theta - local_y * sin_theta + x_offset;
    const double base_y = local_x * sin_theta + local_y * cos_theta + y_offset;
    const float  fused_range = static_cast<float>(std::hypot(base_x, base_y));
    const float  fused_angle = static_cast<float>(std::atan2(base_y, base_x));
    int bucket_index = static_cast<int>(std::floor((fused_angle - ANGLE_MIN) /
                                                   angle_increment_rad_));
    if (bucket_index < 0 || static_cast<size_t>(bucket_index) >= TOTAL_BUCKETS) continue;
    float & current_value = fused_scan.ranges[static_cast<size_t>(bucket_index)];
    if (std::isinf(current_value)) {
      current_value = fused_range;
    } else if (std::fabs(current_value - fused_range) < MERGE_THRESHOLD_M) {
      current_value = 0.5f * (current_value + fused_range);
    } else if (fused_range < current_value) {
      current_value = fused_range;                                 // 取最近者
    }
  }
}

/* ============================================================ */
void LidarFusion::publishFused()
{
  if (angle_increment_rad_ == 0.0 || bucket_count_ == 0) {
    return;  // 还没收到任何一帧，拿不到角分辨率
  }

  const rclcpp::Time now = this->now();
  const bool alive1 = last_scan1_ && (now - last_scan1_time_).seconds() <= scan_timeout_sec_;
  const bool alive2 = last_scan2_ && (now - last_scan2_time_).seconds() <= scan_timeout_sec_;

  /* ---------- 状态变化时打日志 ---------- */
  if (first_state_log_ || alive1 != prev_alive1_ || alive2 != prev_alive2_) {
    if (alive1 && alive2) {
      RCLCPP_INFO(get_logger(), "两台雷达均在线: 融合输出。");
    } else if (alive1) {
      RCLCPP_WARN(get_logger(), "雷达2(%s)无数据: 降级为只输出雷达1。", scan2_topic_.c_str());
    } else if (alive2) {
      RCLCPP_WARN(get_logger(), "雷达1(%s)无数据: 降级为只输出雷达2。", scan1_topic_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "两台雷达均无数据: 暂停发布 %s。", fused_topic_.c_str());
    }
    prev_alive1_ = alive1;
    prev_alive2_ = alive2;
    first_state_log_ = false;
  }

  if (!alive1 && !alive2) {
    return;  // 无可用数据，不发空帧
  }

  LaserScan fused_scan;

  /* 时间戳取最新一帧，方便下游 TF 对齐 */
  if (alive1 && alive2) {
    fused_scan.header.stamp = (last_scan1_time_ >= last_scan2_time_)
        ? last_scan1_->header.stamp : last_scan2_->header.stamp;
  } else if (alive1) {
    fused_scan.header.stamp = last_scan1_->header.stamp;
  } else {
    fused_scan.header.stamp = last_scan2_->header.stamp;
  }

  fused_scan.header.frame_id   = frame_id_;
  fused_scan.angle_min         = -M_PI;
  fused_scan.angle_max         =  M_PI;
  fused_scan.angle_increment   = angle_increment_rad_;

  float range_min = std::numeric_limits<float>::infinity();
  float range_max = 0.0f;
  if (alive1) {
    range_min = std::min(range_min, last_scan1_->range_min);
    range_max = std::max(range_max, last_scan1_->range_max);
  }
  if (alive2) {
    range_min = std::min(range_min, last_scan2_->range_min);
    range_max = std::max(range_max, last_scan2_->range_max);
  }
  fused_scan.range_min = range_min;
  fused_scan.range_max = range_max;

  const auto & ref = alive1 ? last_scan1_ : last_scan2_;   // 任一存活帧作为代表
  fused_scan.scan_time      = ref->scan_time;
  fused_scan.time_increment = (bucket_count_ > 0)
      ? ref->scan_time / static_cast<double>(bucket_count_) : 0.0;

  fused_scan.ranges.assign(bucket_count_, std::numeric_limits<float>::infinity());
  fused_scan.intensities.assign(bucket_count_, 0.0f);

  if (alive1) {
    integrateOneLidar(last_scan1_,
                      std::cos(lidar1_angle_deg_ * M_PI / 180.0),
                      std::sin(lidar1_angle_deg_ * M_PI / 180.0),
                      lidar1_x_offset_m_, lidar1_y_offset_m_, fused_scan);
  }
  if (alive2) {
    integrateOneLidar(last_scan2_,
                      std::cos(lidar2_angle_deg_ * M_PI / 180.0),
                      std::sin(lidar2_angle_deg_ * M_PI / 180.0),
                      lidar2_x_offset_m_, lidar2_y_offset_m_, fused_scan);
  }

  fused_scan_pub_->publish(fused_scan);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFusion>());
  rclcpp::shutdown();
  return 0;
}
