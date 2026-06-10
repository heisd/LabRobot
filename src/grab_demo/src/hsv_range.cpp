// =============================================================================
// hsv_range.cpp
//
// 基于 HSV 颜色阈值的检测抓取节点(整理重写版)。
//
// 与 YOLO / KCF 保持完全一致的统一接口:
//   订阅彩色  : /camera_arm/color/image_raw   (BGR8)
//   订阅深度  : /camera_arm/depth/image_raw   (16UC1, mm)
//   订阅内参  : /gemini_info
//   发布 TF   : camera_arm_depth_optical_frame -> target_frame  (经 TargetTFPublisher)
// 因此 grab_service_node 对 HSV/YOLO/KCF 一视同仁, 无需改动。
//
// 相比旧版的改动:
//   - 修复结构性 bug: message_filters 同步器在构造函数里正确建立(旧版误放在回调里);
//     相机内参订阅修正(旧版 mera_info_sub 拼写/未声明)。
//   - 深度读取 + 针孔反投影 + 广播 TF 统一交给 grab_demo::TargetTFPublisher。
//   - 默认 headless 不弹窗; HSV 阈值改为 ROS 参数。需要实时调参时把 show_image 置 true,
//     会弹出带滑动条的调参窗口(等价旧版的 trackbar 工作流)。
// =============================================================================

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "grab_demo/target_tf_publisher.hpp"  // 统一接口: 像素中心 -> target_frame TF

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <vector>

class HsvRangeNode : public rclcpp::Node
{
public:
  HsvRangeNode() : Node("hsv_range_node")
  {
    // ---- 参数 ----
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera_arm/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera_arm/depth/image_raw");
    info_topic_ = declare_parameter<std::string>("camera_info_topic", "/gemini_info");
    std::string camera_frame = declare_parameter<std::string>(
        "camera_frame", "camera_arm_depth_optical_frame");
    std::string target_frame = declare_parameter<std::string>("target_frame", "target_frame");
    double z_offset = declare_parameter<double>("z_offset", 0.07);

    // HSV 阈值(默认偏红色物体, 与 KCF 自动播种一致, 按需调)
    hue_min_ = declare_parameter<int>("hue_min", 0);
    hue_max_ = declare_parameter<int>("hue_max", 10);
    sat_min_ = declare_parameter<int>("sat_min", 100);
    sat_max_ = declare_parameter<int>("sat_max", 255);
    val_min_ = declare_parameter<int>("val_min", 100);
    val_max_ = declare_parameter<int>("val_max", 255);
    min_area_ = declare_parameter<int>("min_area", 200);

    show_image_ = declare_parameter<bool>("show_image", false);
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);

    // ---- 统一接口: 深度 + 反投影 + 广播 camera_frame->target_frame ----
    tf_pub_.setup(this, camera_frame, target_frame, z_offset);

    // ---- 订阅相机内参 ----
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic_, 1,
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          tf_pub_.setIntrinsics(*msg);
        });

    // ---- 彩色 + 深度 时间近似同步(在构造函数里正确建立) ----
    rgb_sub_.subscribe(this, rgb_topic_);
    depth_sub_.subscribe(this, depth_topic_);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&HsvRangeNode::imageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/detection_image", 1);
    }
    if (show_image_) {
      setupTuningWindow();  // 仅在显式开启时弹出带滑动条的调参窗口
    }

    // 帧流健康监控: 长时间收不到图像就告警(便于排查相机掉线/话题不匹配)
    watchdog_ = create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&HsvRangeNode::watchdog, this));

    // 运行时动态调参(ros2 param set 即时生效), 重点是 z_offset(抓取深度) 与 HSV 阈值
    // —— 与 YOLO/KCF 同一思路。
    param_cb_ = add_on_set_parameters_callback(
        std::bind(&HsvRangeNode::onSetParams, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "hsv_range_node 已启动 (H[%d,%d] S[%d,%d] V[%d,%d], min_area=%d)",
                hue_min_, hue_max_, sat_min_, sat_max_, val_min_, val_max_, min_area_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  // 可选的实时调参窗口(滑动条直接绑定到阈值成员)
  void setupTuningWindow()
  {
    cv::namedWindow("HSV Tuning", cv::WINDOW_NORMAL);
    cv::createTrackbar("Hue Min", "HSV Tuning", &hue_min_, 255);
    cv::createTrackbar("Hue Max", "HSV Tuning", &hue_max_, 255);
    cv::createTrackbar("Sat Min", "HSV Tuning", &sat_min_, 255);
    cv::createTrackbar("Sat Max", "HSV Tuning", &sat_max_, 255);
    cv::createTrackbar("Val Min", "HSV Tuning", &val_min_, 255);
    cv::createTrackbar("Val Max", "HSV Tuning", &val_max_, 255);
  }

  // 动态参数回调: 运行中用 `ros2 param set /color_node <name> <value>` 调整
  rcl_interfaces::msg::SetParametersResult
  onSetParams(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &p : params) {
      const std::string &n = p.get_name();
      if (n == "z_offset") {
        tf_pub_.setZOffset(p.as_double());      // 抓取深度(沿相机光轴)
        RCLCPP_INFO(get_logger(), "z_offset -> %.3f m", p.as_double());
      } else if (n == "hue_min") { hue_min_ = static_cast<int>(p.as_int());
      } else if (n == "hue_max") { hue_max_ = static_cast<int>(p.as_int());
      } else if (n == "sat_min") { sat_min_ = static_cast<int>(p.as_int());
      } else if (n == "sat_max") { sat_max_ = static_cast<int>(p.as_int());
      } else if (n == "val_min") { val_min_ = static_cast<int>(p.as_int());
      } else if (n == "val_max") { val_max_ = static_cast<int>(p.as_int());
      } else if (n == "min_area") { min_area_ = static_cast<int>(p.as_int());
      }
    }
    return result;
  }

  // 帧流健康监控: 长时间收不到图像帧时告警
  void watchdog()
  {
    if (last_frame_count_ == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "尚未收到图像帧 (%s): 相机是否启动? 话题是否匹配?",
                           rgb_topic_.c_str());
      return;
    }
    if (last_frame_count_ == prev_frame_count_) {  // 1s 内无新帧
      if (!stale_warned_) {
        RCLCPP_ERROR(get_logger(), "图像帧中断(>1s无新帧): 相机掉线 / 节点卡死?");
        stale_warned_ = true;
      }
    } else {
      stale_warned_ = false;
    }
    prev_frame_count_ = last_frame_count_;
  }

  // 同步回调: 记录帧到达(健康监控) + 捕获一切异常, 避免回调静默失效
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
  {
    ++last_frame_count_;
    try {
      processFrame(rgb_msg, depth_msg);
    } catch (const cv::Exception &e) {
      ++cb_errors_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "OpenCV 异常(累计 %d): %s", cb_errors_, e.what());
    } catch (const std::exception &e) {
      ++cb_errors_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "检测回调异常(累计 %d): %s", cb_errors_, e.what());
    }
  }

  void processFrame(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
  {
    if (!tf_pub_.intrinsicsReady()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待相机内参 %s ...",
                           info_topic_.c_str());
      return;
    }

    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
    try {
      rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    // BGR -> HSV, 颜色阈值, 形态学去噪
    cv::Mat hsv;
    cv::cvtColor(rgb_ptr->image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(hue_min_, sat_min_, val_min_),
                cv::Scalar(hue_max_, sat_max_, val_max_), mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(mask, mask, kernel);
    cv::dilate(mask, mask, kernel);

    // 找最大色块
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int best_idx = -1;
    double best_area = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
      double a = cv::contourArea(contours[i]);
      if (a >= min_area_ && a > best_area) {
        best_area = a;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx < 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "未检测到符合阈值的颜色目标");
      if (show_image_ || (publish_debug_image_ && hasDebugSub())) {
        drawAndPublish(rgb_ptr->image, contours, -1, cv::Point(-1, -1), mask, -1.0, rgb_msg->header);
      }
      return;
    }

    // 质心
    cv::Moments m = cv::moments(contours[best_idx]);
    if (m.m00 <= 0) return;
    int px = static_cast<int>(m.m10 / m.m00);
    int py = static_cast<int>(m.m01 / m.m00);

    // 统一接口: 像素中心 -> 深度 -> 3D -> target_frame TF
    double dis = tf_pub_.publish(px, py, depth_ptr->image, this->now());

    if (show_image_ || (publish_debug_image_ && hasDebugSub())) {
      drawAndPublish(rgb_ptr->image, contours, best_idx, cv::Point(px, py), mask, dis, rgb_msg->header);
    }

    if (dis <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "目标中心深度无效, 跳过本帧");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "检测到颜色目标 dis=%.3f  质心(%d,%d)  面积=%.0f", dis, px, py, best_area);
  }

  bool hasDebugSub() { return debug_pub_ && debug_pub_->get_subscription_count() > 0; }

  // 画轮廓 + 质心十字 + 距离文字, 发布到话题, 可选弹窗(与 YOLO/KCF 风格一致)
  void drawAndPublish(const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours,
                      int best_idx, const cv::Point &center, const cv::Mat &mask, double dis,
                      const std_msgs::msg::Header &header)
  {
    cv::Mat vis = image.clone();
    if (best_idx >= 0) {
      cv::drawContours(vis, contours, best_idx, cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(vis, center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
      if (dis > 0.0) {  // 在质心旁标注距离(米)
        cv::putText(vis, cv::format("dis=%.3fm", dis),
                    cv::Point(center.x + 12, center.y - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      }
    } else {
      cv::putText(vis, "no target", cv::Point(10, 24),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 200, 255), 2);
    }
    if (publish_debug_image_ && debug_pub_) {
      debug_pub_->publish(*cv_bridge::CvImage(
          header, sensor_msgs::image_encodings::BGR8, vis).toImageMsg());
    }
    if (show_image_) {
      cv::imshow("HSV Detection", vis);
      cv::imshow("HSV Mask", mask);
      cv::waitKey(1);
    }
  }

  // 统一接口
  grab_demo::TargetTFPublisher tf_pub_;

  // 订阅
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // 健康监控状态
  uint64_t last_frame_count_ = 0;   // 收到的帧计数(watchdog 据此判断帧流是否中断)
  uint64_t prev_frame_count_ = 0;
  bool stale_warned_ = false;
  int cb_errors_ = 0;

  // 参数
  std::string rgb_topic_, depth_topic_, info_topic_;
  int hue_min_, hue_max_, sat_min_, sat_max_, val_min_, val_max_, min_area_;
  bool show_image_, publish_debug_image_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HsvRangeNode>());
  rclcpp::shutdown();
  return 0;
}
