// =============================================================================
// closed_loop_grab_node.cpp
//
// 闭环 (闭环检测 / 位置闭环视觉伺服 PBVS) 抓取服务节点。
//
// 与 grab_service_node 的区别:
//   grab_service_node : 开环 —— 只在收到请求时查一次 target_frame, 规划一次, 执行一次。
//                       目标若有偏差(标定误差/深度噪声)无法修正。
//   closed_loop_grab  : 闭环 —— "看 -> 动 -> 再看 -> 再修正" 循环若干次:
//                         1. 查最新 target_frame (识别节点以相机帧率持续刷新);
//                         2. 移动到目标正上方的预抓取点(pre-grasp / hover);
//                         3. 在新视角下重新检测, 目标位置被刷新;
//                         4. 比较本次与上次目标位置, 若变化小于阈值 -> 认为收敛;
//                            否则带着修正量再来一轮(最多 max_iters 轮);
//                         5. 收敛后下降到抓取点, 闭合夹爪, 退回观察点。
//
// 对外接口与 grab_service_node 完全一致 (服务名默认 obj_grab_service,
// 请求里给 target_frame 名), 因此可作为它的"闭环版"直接替换, 上层(Dashboard/VLM)无感。
//
// 识别前端无关: 本节点只读 TF, 不关心识别用的是 yolo_ros / HSV / KCF / VLM 哪一种。
// =============================================================================

#include "rclcpp/rclcpp.hpp"
#include "grab_demo/srv/grab_object.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "lebai_interfaces/srv/set_gripper.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class ClosedLoopGrab : public rclcpp::Node
{
public:
  ClosedLoopGrab() : Node("closed_loop_grab_node")
  {
    RCLCPP_INFO(get_logger(), "closed_loop_grab_node is running");
    node_options_.automatically_declare_parameters_from_overrides(true);

    // ---- 闭环参数 ----
    base_frame_      = declare_parameter<std::string>("base_frame", "base_link");
    look_target_     = declare_parameter<std::string>("look_target", "look");
    max_iters_       = declare_parameter<int>("max_iters", 4);
    pos_tolerance_   = declare_parameter<double>("pos_tolerance", 0.008);   // m, 目标稳定阈值
    approach_height_ = declare_parameter<double>("approach_height", 0.10);  // m, 预抓取悬停高度
    grasp_z_offset_  = declare_parameter<double>("grasp_z_offset", 0.02);   // m, 最终下降补偿(与开环一致)
    settle_sec_      = declare_parameter<double>("settle_sec", 0.6);        // 每次移动后等识别刷新的时间
    vel_scale_       = declare_parameter<double>("velocity_scaling", 0.1);
    plan_time_       = declare_parameter<double>("planning_time", 30.0);
    tf_timeout_      = declare_parameter<double>("tf_timeout", 5.0);        // 首次等待 TF 的超时(s)

    // ---- TF 监听 (持久化; TransformListener 默认开独立线程, 抓取循环里能查到最新 TF) ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- MoveGroup (与 grab_service_node 同样的构造方式) ----
    std::shared_ptr<rclcpp::Node> tmp_ptr(this, [](rclcpp::Node *) {});  // 不接管生命周期
    this_ptr_ = tmp_ptr;
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this_ptr_, PLANNING_GROUP_);
    move_group_->setPoseReferenceFrame(base_frame_);
    end_link_ = move_group_->getEndEffectorLink();
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setGoalPositionTolerance(0.001);
    move_group_->setGoalOrientationTolerance(0.01);
    move_group_->setPlanningTime(plan_time_);

    // ---- 夹爪 ----
    gripper_client_ = create_client<lebai_interfaces::srv::SetGripper>(
        "/io_service/set_gripper_position");
    gripper_client_->wait_for_service();
    gripper_req_ = std::make_shared<lebai_interfaces::srv::SetGripper::Request>();

    // ---- 抓取服务 (默认名与开环版一致, 便于直接替换) ----
    grab_service_ = create_service<grab_demo::srv::GrabObject>(
        "obj_grab_service",
        std::bind(&ClosedLoopGrab::onGrab, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "闭环抓取就绪: max_iters=%d, pos_tol=%.3fm, approach_height=%.3fm, settle=%.1fs",
                max_iters_, pos_tolerance_, approach_height_, settle_sec_);
  }

private:
  // 张开/闭合夹爪 (val: 100=张开, 0=闭合)
  void setGripper(double val)
  {
    gripper_req_->val = val;
    gripper_client_->async_send_request(gripper_req_);
  }

  // 查 base_frame -> target 的平移; 成功返回 true 并填 x/y/z
  bool lookupTarget(const std::string &target, double &x, double &y, double &z, double wait_sec)
  {
    if (!tf_buffer_->canTransform(base_frame_, target, rclcpp::Time(0),
                                  rclcpp::Duration::from_seconds(wait_sec))) {
      return false;
    }
    try {
      auto tfs = tf_buffer_->lookupTransform(base_frame_, target, rclcpp::Time(0),
                                             rclcpp::Duration::from_seconds(wait_sec));
      x = tfs.transform.translation.x;
      y = tfs.transform.translation.y;
      z = tfs.transform.translation.z;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF 查找失败: %s", ex.what());
      return false;
    }
  }

  // 规划并移动到给定位置(姿态沿用当前末端姿态); 成功返回 true
  bool moveTo(double x, double y, double z, const geometry_msgs::msg::Quaternion &ori)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = ori;

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "规划失败: 目标 (%.3f, %.3f, %.3f)", x, y, z);
      return false;
    }
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void onGrab(const std::shared_ptr<grab_demo::srv::GrabObject::Request> req,
              std::shared_ptr<grab_demo::srv::GrabObject::Response> res)
  {
    const std::string target = req->obj_link;
    RCLCPP_INFO(get_logger(), "[闭环] 开始抓取, 目标 TF: %s", target.c_str());

    setGripper(100);  // 张开夹爪

    // 1. 先到观察点 look
    RCLCPP_INFO(get_logger(), "[闭环] 移动到观察点 '%s'", look_target_.c_str());
    move_group_->setNamedTarget(look_target_);
    if (move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "移动到观察点失败, 继续尝试");
    }

    // 末端当前姿态(整个抓取过程保持此姿态, 与开环版一致)
    geometry_msgs::msg::Quaternion ori = move_group_->getCurrentPose(end_link_).pose.orientation;

    // 2. 闭环修正循环
    double px = 0, py = 0, pz = 0;     // 上一轮目标
    bool have_prev = false;
    double tx = 0, ty = 0, tz = 0;     // 本轮目标
    bool converged = false;

    for (int iter = 0; iter < max_iters_; ++iter) {
      // 等识别节点在当前视角下刷新 target_frame
      std::this_thread::sleep_for(std::chrono::duration<double>(settle_sec_));

      double wait = (iter == 0) ? tf_timeout_ : 1.0;
      if (!lookupTarget(target, tx, ty, tz, wait)) {
        if (iter == 0) {
          RCLCPP_ERROR(get_logger(), "[闭环] 无法获取 TF: %s -> %s",
                       base_frame_.c_str(), target.c_str());
          res->success = false;
          res->message = "TF transform not available";
          return;
        }
        RCLCPP_WARN(get_logger(), "[闭环] 第 %d 轮丢失目标, 用上一轮结果", iter);
        break;
      }

      // 收敛判定: 目标位置相比上一轮几乎不动 -> 已对准
      if (have_prev) {
        double d = std::sqrt((tx - px) * (tx - px) + (ty - py) * (ty - py) +
                             (tz - pz) * (tz - pz));
        RCLCPP_INFO(get_logger(), "[闭环] 第 %d 轮: 目标 (%.3f, %.3f, %.3f), Δ=%.4fm",
                    iter, tx, ty, tz, d);
        if (d < pos_tolerance_) {
          RCLCPP_INFO(get_logger(), "[闭环] 目标已收敛 (Δ=%.4f < %.4f), 进入抓取", d, pos_tolerance_);
          converged = true;
          break;
        }
      } else {
        RCLCPP_INFO(get_logger(), "[闭环] 第 %d 轮: 目标 (%.3f, %.3f, %.3f)", iter, tx, ty, tz);
      }

      // 移动到目标正上方的预抓取点, 带着修正量靠近
      if (!moveTo(tx, ty, tz + approach_height_, ori)) {
        RCLCPP_ERROR(get_logger(), "[闭环] 移动到预抓取点失败");
        res->success = false;
        res->message = "Pre-grasp planning/execution failed";
        return;
      }
      px = tx; py = ty; pz = tz;
      have_prev = true;
    }

    if (!converged) {
      RCLCPP_WARN(get_logger(), "[闭环] 达到最大轮数仍未完全收敛, 用当前最优解抓取");
    }

    // 3. 最终下降到抓取点 (再查一次 TF, 用最新结果)
    if (!lookupTarget(target, tx, ty, tz, 1.0)) {
      RCLCPP_WARN(get_logger(), "[闭环] 最终查 TF 失败, 使用上一轮目标");
      tx = px; ty = py; tz = pz;
    }
    RCLCPP_INFO(get_logger(), "[闭环] 下降抓取: (%.3f, %.3f, %.3f)", tx, ty, tz + grasp_z_offset_);
    if (!moveTo(tx, ty, tz + grasp_z_offset_, ori)) {
      RCLCPP_ERROR(get_logger(), "[闭环] 下降到抓取点失败");
      res->success = false;
      res->message = "Final approach failed";
      return;
    }

    // 4. 闭合夹爪
    std::this_thread::sleep_for(800ms);
    setGripper(0);
    std::this_thread::sleep_for(800ms);

    // 5. 退回观察点
    RCLCPP_INFO(get_logger(), "[闭环] 抓取完成, 退回观察点 '%s'", look_target_.c_str());
    move_group_->setNamedTarget(look_target_);
    move_group_->move();

    res->success = true;
    res->message = converged ? "Grasped (closed-loop converged)"
                             : "Grasped (max iters reached)";
  }

  // 成员
  rclcpp::NodeOptions node_options_;
  std::shared_ptr<rclcpp::Node> this_ptr_;
  const std::string PLANNING_GROUP_ = "manipulator";
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string end_link_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedPtr gripper_client_;
  std::shared_ptr<lebai_interfaces::srv::SetGripper::Request> gripper_req_;
  rclcpp::Service<grab_demo::srv::GrabObject>::SharedPtr grab_service_;

  std::string base_frame_, look_target_;
  int max_iters_;
  double pos_tolerance_, approach_height_, grasp_z_offset_, settle_sec_;
  double vel_scale_, plan_time_, tf_timeout_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopGrab>());
  rclcpp::shutdown();
  return 0;
}
