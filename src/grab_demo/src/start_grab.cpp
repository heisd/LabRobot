#include "rclcpp/rclcpp.hpp"
#include "grab_demo/srv/grab_object.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "lebai_interfaces/srv/set_gripper.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
using namespace std::chrono_literals;
int main(int argc, char* argv[])
{
  
  std::string target_tf_frame=argv[1];
  rclcpp::init(argc, argv);
  // 创建一个客户端节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client_node");
  // 创建一个客户端
  rclcpp::Client<grab_demo::srv::GrabObject>::SharedPtr client =node->create_client<grab_demo::srv::GrabObject>("/obj_grab_service");
  // 创建一个请求
  auto request =std::make_shared<grab_demo::srv::GrabObject::Request>();
  request->obj_link=target_tf_frame;
  std::chrono::seconds myPause = 1s;
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "SERVICE NOT AVAILABLE, waiting again...");
  } 
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sucess");
  } else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_cap_full_name");
  }
  rclcpp::shutdown();
  return 0;
}