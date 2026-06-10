#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 客户端库
#include <moveit/moveit_cpp/moveit_cpp.h>  // MoveIt2 C++ API
#include <moveit/moveit_cpp/planning_component.h>  // MoveIt2 规划组件
#include <moveit/move_group_interface/move_group_interface.h>  // MoveGroupInterface 类
#include "iostream"  // 输入输出流
#include <string>  // 字符串处理库

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_cpp_demo");  // 创建全局日志记录器

int main(int argc, char const *argv[])
{      
    RCLCPP_INFO(LOGGER, "arm_ik_demo");  // 打印日志信息，表示程序开始执行
    rclcpp::init(argc,argv);  // 初始化ROS节点
    rclcpp::NodeOptions node_options;  // 创建节点选项对象
    geometry_msgs::msg::Pose current_pose;  // 创建存储当前位姿的变量
    node_options.automatically_declare_parameters_from_overrides(true);  // 设置节点选项，自动声明参数
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);  // 创建MoveGroupInterface的节点

    rclcpp::executors::SingleThreadedExecutor executor;  // 创建单线程执行器
    executor.add_node(move_group_node);  // 将MoveGroupInterface的节点添加到执行器中
    std::thread([&executor]() { executor.spin(); }).detach();  // 在单独的线程中运行执行器

    static const std::string PLANNING_GROUP = "manipulator";  // 设置规划组名称
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;  // 创建规划路径对象
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node,PLANNING_GROUP);  // 创建MoveGroupInterface对象，并指定规划组
    move_group.setPoseReferenceFrame("base_link");  // 设置姿态参考坐标系
    std::string end_link=move_group.getEndEffectorLink();  // 获取末端执行器的链接名称

    geometry_msgs::msg::Pose target_pose;  // 创建目标位姿变量

    move_group.setNamedTarget("look");  // 设置命名的目标位置
    move_group.move();  // 执行规划和移动操作

    move_group.setStartStateToCurrentState();  // 将当前状态设置为起始状态
    move_group.setGoalTolerance(0.02);  // 设置目标容差值
    sleep(1);  // 等待1秒

    target_pose.position.x= 0.6;  // 目标位姿的x坐标
    target_pose.position.y= 0.17;  // 目标位姿的y坐标
    target_pose.position.z= 0.46;  // 目标位姿的z坐标

    target_pose.orientation.x= 0.16;  // 目标位姿的四元数x分量
    target_pose.orientation.y= 0.05;  // 目标位姿的四元数y分量
    target_pose.orientation.z= 0.013;  // 目标位姿的四元数z分量
    target_pose.orientation.w= 0.98;  // 目标位姿的四元数w分量

    move_group.setPoseTarget(target_pose);  // 设置目标位姿
    move_group.setPlanningTime(30);  // 设置规划时间
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);  // 进行路径规划
    if(success) move_group.execute(my_plan);  // 执行规划路径
    sleep(1);  // 等待1秒

    move_group.setNamedTarget("look");  // 设置命名的目标位置
    move_group.move();  // 执行规划和移动操作

    rclcpp::shutdown();  // 关闭ROS节点
    return 0;  // 程序正常结束
}
