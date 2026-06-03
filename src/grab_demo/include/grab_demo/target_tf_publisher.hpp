// =============================================================================
// target_tf_publisher.hpp
//
// 抓取目标的"统一接口": 把不同视觉算法(HSV / YOLO / KCF ...)算出的目标像素中心,
// 统一转换成 base 可用的 target_frame TF。
//
// 各算法只需要做一件事 —— 在彩色图上得到目标的像素中心 (px, py),
// 之后的「读取深度 -> 针孔反投影 -> 广播 camera_frame->target_frame」全部由本类完成。
// 这样 HSV / YOLO / KCF 三种算法对 grab_service_node 暴露的接口完全一致(都是 target_frame),
// 抓取服务无需为不同算法做任何改动。
// =============================================================================
#ifndef GRAB_DEMO_TARGET_TF_PUBLISHER_HPP_
#define GRAB_DEMO_TARGET_TF_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace grab_demo
{

class TargetTFPublisher
{
public:
  // 绑定到节点并设置坐标系与 Z 补偿(与 HSV/YOLO 保持一致, 默认 0.07m)
  void setup(rclcpp::Node *node, const std::string &camera_frame,
             const std::string &target_frame, double z_offset)
  {
    camera_frame_ = camera_frame;
    target_frame_ = target_frame;
    z_offset_ = z_offset;
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }

  // 从 CameraInfo 设置相机内参(与 HSV/YOLO 同样的校验: K 非零才生效)
  void setIntrinsics(const sensor_msgs::msg::CameraInfo &info)
  {
    bool k_valid = false;
    for (size_t i = 0; i < info.k.size(); ++i) {
      if (info.k[i] != 0) { k_valid = true; break; }
    }
    if (!k_valid) return;
    fx_ = info.k[0];
    cx_ = info.k[2];
    fy_ = info.k[4];
    cy_ = info.k[5];
    ready_ = true;
  }

  bool intrinsicsReady() const { return ready_; }

  // 中心 (2r+1)x(2r+1) 窗口内非零深度的中值, 单位米; 无有效深度返回 0
  static double medianDepth(const cv::Mat &depth, int px, int py, int r = 5)
  {
    std::vector<ushort> vals;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        int x = px + dx, y = py + dy;
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) continue;
        ushort v = depth.at<ushort>(y, x);
        if (v > 0) vals.push_back(v);
      }
    }
    if (vals.empty()) return 0.0;
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return vals[vals.size() / 2] / 1000.0;  // mm -> m
  }

  // 给定目标像素中心 + 深度图, 反投影并广播 TF。
  // 返回测得的距离(米), <=0 表示深度无效/内参未就绪, 未广播。
  double publish(int px, int py, const cv::Mat &depth, const rclcpp::Time &stamp)
  {
    if (!ready_) return -1.0;
    px = std::clamp(px, 0, depth.cols - 1);
    py = std::clamp(py, 0, depth.rows - 1);
    double dis = medianDepth(depth, px, py, 5);
    if (dis <= 0.0) return -1.0;

    double x = (px - cx_) / fx_ * dis;
    double y = (py - cy_) / fy_ * dis;
    double z = dis + z_offset_;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = camera_frame_;
    tf.child_frame_id = target_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w = 1.0;  // 仅给位置, 姿态由抓取服务决定
    tf_pub_->sendTransform(tf);
    return dis;
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::string camera_frame_, target_frame_;
  double z_offset_ = 0.07;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  bool ready_ = false;
};

}  // namespace grab_demo

#endif  // GRAB_DEMO_TARGET_TF_PUBLISHER_HPP_
