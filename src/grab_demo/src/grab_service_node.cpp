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
class obj_grab :public rclcpp::Node
{
private:
    rclcpp::NodeOptions node_options;
    geometry_msgs::msg::Pose current_pose;
    std::shared_ptr<lebai_interfaces::srv::SetGripper::Request> p_request;
    std::string target_link;
    rclcpp::Client<lebai_interfaces::srv::SetGripper>::SharedPtr lebai_pclient;
    rclcpp::Service<grab_demo::srv::GrabObject>::SharedPtr grab_service;
    // moveit::planning_interface::MoveGroupInterface *move_group;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::msg::Pose target_pose;
    std::string end_link;
    std::shared_ptr<rclcpp::Node> this_ptr;
public:
    // Debug 发现没有这个节点 在ROS2 node list 上
    // 证明这个没有运行到这里
    obj_grab():Node("object_grab_node")
    {
        // 观测节点运行成功没有
        RCLCPP_INFO(this->get_logger(),"obj_grab_node is running");
        node_options.automatically_declare_parameters_from_overrides(true); 
        std::shared_ptr<rclcpp::Node> tmp_ptr(this);
        this_ptr=tmp_ptr;       
        move_group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(this_ptr,PLANNING_GROUP);
        lebai_pclient=this->create_client<lebai_interfaces::srv::SetGripper>("/io_service/set_gripper_position");
        lebai_pclient->wait_for_service();
        p_request=std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
        grab_service=this->create_service<grab_demo::srv::GrabObject>("obj_grab_service",std::bind(&obj_grab::start_grab,this,std::placeholders::_1,std::placeholders::_2));
        move_group->setPoseReferenceFrame("base_link");//设置机器人末端执行器的参考坐标系
        std::string end_link=move_group->getEndEffectorLink();// 获取机器人末端执行器名称
        move_group->setMaxVelocityScalingFactor(0.1);// 设置最大速度缩放因子
        move_group->setGoalPositionTolerance(0.001);// 设置目标位置允许误差
        move_group->setGoalOrientationTolerance(0.01); // 设置目标方向允许误差
        end_link=move_group->getEndEffectorLink();// 获取机器人末端执行器名称
    }
    void start_grab(const std::shared_ptr<grab_demo::srv::GrabObject::Request> req,std::shared_ptr<grab_demo::srv::GrabObject::Response> res);

};

void obj_grab::start_grab(const std::shared_ptr<grab_demo::srv::GrabObject::Request> req,std::shared_ptr<grab_demo::srv::GrabObject::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "开始抓取任务，目标TF: %s", req->obj_link.c_str());
    
    p_request->val=100;
    auto result_=lebai_pclient->async_send_request(p_request);//100 张开夹爪

    tf2_ros::Buffer buf(this->get_clock());
    tf2_ros::TransformListener listener(buf);
    
    // 检查 TF 变换是否可用
    RCLCPP_INFO(this->get_logger(), "等待 TF 变换: base_link -> %s", req->obj_link.c_str());
    if (!buf.canTransform("base_link", req->obj_link, rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0))) {
        RCLCPP_ERROR(this->get_logger(), "无法获取 TF 变换: base_link -> %s", req->obj_link.c_str());
        RCLCPP_ERROR(this->get_logger(), "请检查:");
        RCLCPP_ERROR(this->get_logger(), "1. %s 是否正在发布 TF", req->obj_link.c_str());
        RCLCPP_ERROR(this->get_logger(), "2. TF 变换链是否完整 (base_link -> camera -> %s)", req->obj_link.c_str());
        res->success = false;
        res->message = "TF transform not available";
        return;
    }
    
    geometry_msgs::msg::TransformStamped tfs;
    try {
        tfs = buf.lookupTransform("base_link", req->obj_link, rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF 查找失败: %s", ex.what());
        res->success = false;
        res->message = std::string("TF lookup failed: ") + ex.what();
        return;
    }
    
    geometry_msgs::msg::Pose cur_pose=move_group->getCurrentPose(end_link).pose;

    target_pose.position.x=tfs.transform.translation.x;
    target_pose.position.y=tfs.transform.translation.y;
    target_pose.position.z=tfs.transform.translation.z;
    
    // 设置目标姿态
    // 默认使用当前位置的姿态（保持当前方向）
    // 如果需要垂直向下抓取，可以修改为下面的姿态
    target_pose.orientation.x = cur_pose.orientation.x;
    target_pose.orientation.y = cur_pose.orientation.y;
    target_pose.orientation.z = cur_pose.orientation.z;
    target_pose.orientation.w = cur_pose.orientation.w;
    
    // 可选：使用垂直向下姿态（Z轴指向下方，适合抓取）
    // 如果规划失败，可以尝试取消下面的注释
    // target_pose.orientation.x = 1.0;  // 绕X轴180度，Z轴向下
    // target_pose.orientation.y = 0.0;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "目标位置: x=%.3f, y=%.3f, z=%.3f", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "目标姿态: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                target_pose.orientation.x, target_pose.orientation.y, 
                target_pose.orientation.z, target_pose.orientation.w);
    
    RCLCPP_INFO(this->get_logger(), "移动到观察点 'look'");
    move_group->setNamedTarget("look");//先移动到预先设定好的look 目标点观察物体
    moveit::core::MoveItErrorCode move_result = move_group->move();
    if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "移动到观察点失败，错误代码: %d", move_result.val);
    }
    
    move_group->setStartStateToCurrentState();
    sleep(1);

    RCLCPP_INFO(this->get_logger(), "规划到目标位置...");
    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(30);
    moveit::core::MoveItErrorCode plan_result = move_group->plan(my_plan);
    
    if(plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "规划成功，开始执行");
        moveit::core::MoveItErrorCode execute_result = move_group->execute(my_plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "执行成功，到达目标位置");
            res->success = true;
            res->message = "Successfully moved to target";
        } else {
            RCLCPP_ERROR(this->get_logger(), "执行失败，错误代码: %d", execute_result.val);
            res->success = false;
            res->message = "Execution failed";
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "规划失败，错误代码: %d", plan_result.val);
        RCLCPP_ERROR(this->get_logger(), "可能的原因:");
        RCLCPP_ERROR(this->get_logger(), "1. 目标位置超出工作空间");
        RCLCPP_ERROR(this->get_logger(), "2. 目标位置有碰撞");
        RCLCPP_ERROR(this->get_logger(), "3. 目标姿态不可达");
        res->success = false;
        res->message = "Planning failed";
        return;
    }
    
    sleep(1);
    p_request->val=0;
    result_=lebai_pclient->async_send_request(p_request);//0 闭合夹爪
    sleep(1);
    
    RCLCPP_INFO(this->get_logger(), "返回观察点 'look'");
    move_group->setNamedTarget("look");//夹取后回到观察点look，也可以夹取后移动到其他的点，在stdf文件里面新增设定点
    move_group->move();
}


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<obj_grab>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}