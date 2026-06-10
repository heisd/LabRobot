#ifndef LIDAR_FUSION_HPP_
#define LIDAR_FUSION_HPP_

#include <memory>
#include <string>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief 订阅两台 2D 激光雷达的 LaserScan，融合成 360° LaserScan。
 *
 * 两台雷达**各自独立订阅**并缓存最新一帧，由定时器按固定频率输出：
 *   - 两台都在线（最近一帧在 scan_timeout_sec 内）→ 融合两台；
 *   - 只有一台在线 → 仅输出存活的那一台（仍按其外参投影到基坐标系）；
 *   - 两台都掉线 → 暂停发布，不发空帧。
 *
 * 这样单台雷达故障时不会让整个 /scan 停掉。可通过参数或 launch 配置各
 * Topic 名、frame_id、两台雷达外参（旋转角 + XY 平移）、超时与发布频率。
 */
class LidarFusion : public rclcpp::Node
{
public:
  explicit LidarFusion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarFusion() override = default;

private:
  /* ---------- 回调 ---------- */
  void scan1Cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void scan2Cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void publishFused();

  /** 把一台雷达的点按其外参投影进 fused_scan 的环形桶。 */
  void integrateOneLidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
                         double cos_theta, double sin_theta,
                         double x_offset, double y_offset,
                         sensor_msgs::msg::LaserScan & fused_scan) const;

  /** 首次收到任意一帧时缓存角分辨率与桶数。 */
  void cacheGeometry(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  /* ---------- ROS 接口 ---------- */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    fused_scan_pub_;
  rclcpp::TimerBase::SharedPtr                                 timer_;

  /* ---------- 参数 ---------- */
  std::string frame_id_;
  std::string fused_topic_;   // 发布 topic（默认 "scan"）
  std::string scan1_topic_;
  std::string scan2_topic_;

  double  lidar1_angle_deg_{0.0};   // 雷达 1 旋转角（度）
  double  lidar1_x_offset_m_{0.0};  // 雷达 1 X 平移（米）
  double  lidar1_y_offset_m_{0.0};  // 雷达 1 Y 平移（米）
  double  lidar2_angle_deg_{0.0};   // 雷达 2 旋转角
  double  lidar2_x_offset_m_{0.0};  // 雷达 2 X 平移
  double  lidar2_y_offset_m_{0.0};  // 雷达 2 Y 平移
  double  scan_timeout_sec_{0.5};   // 多久没收到帧就认为该雷达掉线
  double  publish_rate_hz_{12.0};   // 融合结果发布频率

  /* ---------- 运行时缓存 ---------- */
  double   angle_increment_rad_{0.0};   // 单束角度分辨率 (rad)
  size_t   bucket_count_{0};            // 360° 被划分的桶数

  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan1_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan2_;
  rclcpp::Time last_scan1_time_;
  rclcpp::Time last_scan2_time_;

  /* ---------- 状态日志去抖 ---------- */
  bool prev_alive1_{false};
  bool prev_alive2_{false};
  bool first_state_log_{true};
};

#endif  // LIDAR_FUSION_HPP_
