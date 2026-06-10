// =============================================================================
// kcf_track_node.cpp
//
// 基于 KCF (Kernelized Correlation Filter) 的视觉跟踪抓取节点。
//
// 说明: KCF 是"跟踪"算法(cv::TrackerKCF), 不是逐帧检测器 —— 它需要一个初始目标框,
// 之后逐帧跟踪。本节点的初始框来源(按优先级):
//   1) 话题 ~/select_bbox (sensor_msgs/RegionOfInterest): 运行时手动框选,
//      Dashboard 在跟踪画面上拖拽框选后发布到这里, 收到即重新播种;
//   2) 参数 init_bbox = [x, y, w, h] (w,h>0 时使用), 只用于第一次播种;
//   3) 否则用 HSV 颜色阈值找最大色块自动播种(无需手动框选, headless 友好);
//   并提供 ~/reinit 服务(std_srvs/Trigger)随时强制重新播种; 跟丢时自动回到 HSV 重新播种。
//
// 接口与 HSV / YOLO 完全一致(统一接口):
//   订阅彩色  : /camera_arm/color/image_raw   (BGR8)
//   订阅深度  : /camera_arm/depth/image_raw   (16UC1, mm)
//   订阅内参  : /gemini_info
//   发布 TF   : camera_arm_depth_optical_frame -> target_frame  (经 TargetTFPublisher)
// 因此 grab_service_node 无需改动即可复用。默认不弹窗(headless 安全)。
// =============================================================================

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "grab_demo/target_tf_publisher.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <vector>

class KcfTrackNode : public rclcpp::Node
{
public:
  KcfTrackNode() : Node("kcf_track_node")
  {
    // ---- 参数 ----
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera_arm/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera_arm/depth/image_raw");
    info_topic_ = declare_parameter<std::string>("camera_info_topic", "/gemini_info");
    std::string camera_frame = declare_parameter<std::string>(
        "camera_frame", "camera_arm_depth_optical_frame");
    std::string target_frame = declare_parameter<std::string>("target_frame", "target_frame");
    double z_offset = declare_parameter<double>("z_offset", 0.07);

    // 初始框(可选): [x, y, w, h], w/h>0 时作为第一次播种
    init_bbox_ = declare_parameter<std::vector<int64_t>>("init_bbox",
                                                         std::vector<int64_t>{0, 0, 0, 0});

    // HSV 自动播种阈值(默认偏红色物体, 按需改)
    hue_min_ = declare_parameter<int>("hue_min", 0);
    hue_max_ = declare_parameter<int>("hue_max", 10);
    sat_min_ = declare_parameter<int>("sat_min", 100);
    sat_max_ = declare_parameter<int>("sat_max", 255);
    val_min_ = declare_parameter<int>("val_min", 100);
    val_max_ = declare_parameter<int>("val_max", 255);
    min_area_ = declare_parameter<int>("min_area", 400);

    reinit_on_loss_ = declare_parameter<bool>("reinit_on_loss", true);
    show_image_ = declare_parameter<bool>("show_image", false);
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);

    // ---- 统一接口: 像素 -> 深度 -> 3D -> target_frame TF ----
    tf_pub_.setup(this, camera_frame, target_frame, z_offset);

    // ---- 订阅 ----
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic_, 1,
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          tf_pub_.setIntrinsics(*msg);
        });

    rgb_sub_.subscribe(this, rgb_topic_);
    depth_sub_.subscribe(this, depth_topic_);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&KcfTrackNode::imageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // ~/reinit 服务: 强制下一帧重新播种
    reinit_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reinit",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          tracking_ = false;
          have_manual_bbox_ = false;        // 丢弃未消费的手动框, 回到 HSV/init_bbox
          use_init_bbox_ = have_init_bbox();
          res->success = true;
          res->message = "KCF 将在下一帧重新播种";
          RCLCPP_INFO(get_logger(), "收到 reinit 请求, 重新播种跟踪目标");
        });

    // ~/select_bbox 话题: 运行时手动框选(Dashboard 在跟踪画面上拖拽框选)。
    // 坐标为原始图像像素; 收到即放弃当前跟踪, 下一帧用该框播种。
    select_sub_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
        "~/select_bbox", 1,
        [this](const sensor_msgs::msg::RegionOfInterest::ConstSharedPtr msg) {
          if (msg->width == 0 || msg->height == 0) {
            RCLCPP_WARN(get_logger(), "忽略空的手动框选 (w=%u h=%u)", msg->width, msg->height);
            return;
          }
          manual_bbox_ = cv::Rect(static_cast<int>(msg->x_offset),
                                  static_cast<int>(msg->y_offset),
                                  static_cast<int>(msg->width),
                                  static_cast<int>(msg->height));
          have_manual_bbox_ = true;
          tracking_ = false;   // 默认单线程执行器, 与图像回调串行, 无需加锁
          RCLCPP_INFO(get_logger(), "收到手动框选 [x=%d y=%d w=%d h=%d], 将重新播种",
                      manual_bbox_.x, manual_bbox_.y, manual_bbox_.width, manual_bbox_.height);
        });

    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/tracking_image", 1);
    }

    // 帧流健康监控: 长时间收不到图像就告警(便于排查相机掉线/话题不匹配)
    watchdog_ = create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&KcfTrackNode::watchdog, this));

    // 运行时动态调参(ros2 param set 即时生效), 重点是 z_offset(抓取深度), 以及
    // HSV 自动播种阈值/最小面积/跟丢是否重播种 —— 与 YOLO 桥接节点同一思路。
    param_cb_ = add_on_set_parameters_callback(
        std::bind(&KcfTrackNode::onSetParams, this, std::placeholders::_1));

    use_init_bbox_ = have_init_bbox();
    RCLCPP_INFO(get_logger(), "kcf_track_node 已启动 (init_bbox=%s, z_offset=%.3f)",
                use_init_bbox_ ? "指定" : "HSV自动播种", z_offset);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  bool have_init_bbox() const
  {
    return init_bbox_.size() == 4 && init_bbox_[2] > 0 && init_bbox_[3] > 0;
  }

  // 动态参数回调: 运行中用 `ros2 param set /kcf_node <name> <value>` 调整
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
      } else if (n == "reinit_on_loss") { reinit_on_loss_ = p.as_bool();
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
        RCLCPP_ERROR(get_logger(),
                     "图像帧中断(>1s无新帧): 相机掉线 / 节点卡死? 已停止跟踪输出");
        stale_warned_ = true;
        tracking_ = false;   // 停止跟踪, 下次有帧再重新播种
      }
    } else {
      stale_warned_ = false;
    }
    prev_frame_count_ = last_frame_count_;
  }

  // 用 HSV 颜色阈值找最大色块, 返回其外接矩形; 找不到返回空 Rect
  cv::Rect hsvLargestBlob(const cv::Mat &bgr)
  {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(hue_min_, sat_min_, val_min_),
                cv::Scalar(hue_max_, sat_max_, val_max_), mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(mask, mask, kernel);
    cv::dilate(mask, mask, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = 0.0;
    cv::Rect best;
    for (const auto &c : contours) {
      double a = cv::contourArea(c);
      if (a >= min_area_ && a > best_area) {
        best_area = a;
        best = cv::boundingRect(c);
      }
    }
    return best;
  }

  // 尝试播种 KCF 跟踪器; 成功返回 true
  bool seedTracker(const cv::Mat &bgr)
  {
    cv::Rect roi;
    if (have_manual_bbox_) {
      // Dashboard 手动框选优先, 同样只消费一次
      roi = manual_bbox_ & cv::Rect(0, 0, bgr.cols, bgr.rows);
      have_manual_bbox_ = false;
    } else if (use_init_bbox_) {
      roi = cv::Rect(static_cast<int>(init_bbox_[0]), static_cast<int>(init_bbox_[1]),
                     static_cast<int>(init_bbox_[2]), static_cast<int>(init_bbox_[3]));
      roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);  // 裁剪到图像内
      use_init_bbox_ = false;  // 初始框只用一次, 之后跟丢用 HSV 重新播种
    } else {
      roi = hsvLargestBlob(bgr);
    }
    if (roi.width <= 0 || roi.height <= 0) {
      return false;  // 这一帧没找到可播种的目标
    }
    tracker_ = cv::TrackerKCF::create();
    tracker_->init(bgr, roi);
    bbox_ = roi;
    tracking_ = true;
    RCLCPP_INFO(get_logger(), "KCF 已播种, 初始框 [x=%d y=%d w=%d h=%d]",
                roi.x, roi.y, roi.width, roi.height);
    return true;
  }

  // 同步回调: 记录帧到达(健康监控用) + 捕获一切异常, 避免回调静默失效
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
                            "跟踪回调异常(累计 %d): %s", cb_errors_, e.what());
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

    if (!tracking_) {
      // 还没有跟踪目标: 尝试播种
      if (!seedTracker(rgb_ptr->image)) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "等待可跟踪目标(HSV 未找到色块或初始框无效)");
        if (show_image_ || (publish_debug_image_ && hasDebugSub())) {
          drawAndPublish(rgb_ptr->image, false, -1.0, rgb_msg->header);
        }
        return;
      }
    } else {
      // 已在跟踪: 更新
      cv::Rect box = bbox_;
      bool ok = tracker_->update(rgb_ptr->image, box);
      if (ok) {
        bbox_ = box;
      } else {
        tracking_ = false;
        RCLCPP_WARN(get_logger(), "KCF 跟踪丢失%s", reinit_on_loss_ ? ", 将重新播种" : "");
        if (!reinit_on_loss_) return;
        // 本帧立刻尝试用 HSV 重新播种
        use_init_bbox_ = false;
        if (!seedTracker(rgb_ptr->image)) return;
      }
    }

    // 跟踪框中心 -> 统一接口发布 target_frame
    int px = bbox_.x + bbox_.width / 2;
    int py = bbox_.y + bbox_.height / 2;
    double dis = tf_pub_.publish(px, py, depth_ptr->image, this->now());

    if (show_image_ || (publish_debug_image_ && hasDebugSub())) {
      drawAndPublish(rgb_ptr->image, true, dis, rgb_msg->header);
    }

    if (dis <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "跟踪框中心深度无效, 跳过本帧");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "KCF 跟踪中 dis=%.3f  框中心(%d,%d)", dis, px, py);
  }

  bool hasDebugSub() { return debug_pub_ && debug_pub_->get_subscription_count() > 0; }

  void drawAndPublish(const cv::Mat &image, bool has_target, double dis,
                      const std_msgs::msg::Header &header)
  {
    cv::Mat vis = image.clone();
    if (has_target) {
      cv::rectangle(vis, bbox_, cv::Scalar(0, 0, 255), 2);
      int cxp = bbox_.x + bbox_.width / 2;
      int cyp = bbox_.y + bbox_.height / 2;
      cv::drawMarker(vis, cv::Point(cxp, cyp), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
      std::string label = "KCF tracking";
      if (dis > 0.0) label += cv::format("  dis=%.3fm", dis);  // 框上标注距离
      cv::putText(vis, label, cv::Point(bbox_.x, std::max(0, bbox_.y - 6)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    } else {
      cv::putText(vis, "seeking target...", cv::Point(10, 24),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 200, 255), 2);
    }
    if (publish_debug_image_ && debug_pub_) {
      debug_pub_->publish(*cv_bridge::CvImage(
          header, sensor_msgs::image_encodings::BGR8, vis).toImageMsg());
    }
    if (show_image_) {
      cv::imshow("KCF Tracking", vis);
      cv::waitKey(1);
    }
  }

  // 统一接口
  grab_demo::TargetTFPublisher tf_pub_;

  // 订阅 / 服务
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr select_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reinit_srv_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // 跟踪器状态
  cv::Ptr<cv::TrackerKCF> tracker_;
  cv::Rect bbox_;
  bool tracking_ = false;
  bool use_init_bbox_ = false;
  cv::Rect manual_bbox_;           // Dashboard 手动框选(~/select_bbox), 一次性
  bool have_manual_bbox_ = false;

  // 健康监控状态
  uint64_t last_frame_count_ = 0;   // 收到的帧计数(watchdog 据此判断帧流是否中断)
  uint64_t prev_frame_count_ = 0;
  bool stale_warned_ = false;
  int cb_errors_ = 0;

  // 参数
  std::string rgb_topic_, depth_topic_, info_topic_;
  std::vector<int64_t> init_bbox_;
  int hue_min_, hue_max_, sat_min_, sat_max_, val_min_, val_max_, min_area_;
  bool reinit_on_loss_, show_image_, publish_debug_image_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KcfTrackNode>());
  rclcpp::shutdown();
  return 0;
}
