#include "rclcpp/rclcpp.hpp"
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_cpp_demo");

int main(int argc, char const *argv[])
{      
    // 初始化ROS节点
    RCLCPP_INFO(LOGGER, "arm_fk_demo");
    rclcpp::init(argc,argv);

    // 创建节点选项
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 创建MoveGroupInterface的节点
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // 创建执行器，并将MoveGroupInterface的节点添加到执行器中
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);

    // 在单独的线程中运行执行器
    std::thread([&executor]() { executor.spin(); }).detach();

    // 设置规划组名称
    static const std::string PLANNING_GROUP = "manipulator";

    // 设置目标关节角度
    std::vector<double> joint_target_angle(6);
    joint_target_angle[0]=-1.01;
    joint_target_angle[1]=-2.14;
    joint_target_angle[2]=-1.72;
    joint_target_angle[3]=-0.77;
    joint_target_angle[4]=1.9;
    joint_target_angle[5]=3.68;

    // 创建MoveGroupInterface对象，并指定规划组
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);       

    // 设置关节目标值
    move_group.setJointValueTarget(joint_target_angle);

    // 执行规划和移动操作
    move_group.move();

    // 等待1秒
    sleep(1);

    // 设置命名的目标位置
    move_group.setNamedTarget("look");
    
    // 执行规划和移动操作
    move_group.move();

    // 关闭ROS节点
    rclcpp::shutdown();
    return 0;
}
