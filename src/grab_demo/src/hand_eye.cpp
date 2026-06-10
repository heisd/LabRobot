#include "grab_demo/hand_eye.hpp"

std::vector<cv::Mat> R_gripper2base, t_gripper2base;
std::vector<cv::Mat> R_target2cam, t_target2cam;

bool move_flag=0;
int hand_eye_cout=18;

void charuco_pose_callback(const geometry_msgs::msg::Pose::ConstPtr msg)
{
    if(move_flag)
    {
            move_flag=0;
            cv::Mat temp1;
            Eigen::Quaterniond q1(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
            cv::eigen2cv(q1.toRotationMatrix(),temp1);
            R_target2cam.push_back(temp1);
            cv::Mat temp_tmat(3,1,CV_64FC1);

            temp_tmat.at<double>(0,0)=msg->position.x;
            temp_tmat.at<double>(1,0)=msg->position.y;
            temp_tmat.at<double>(2,0)=msg->position.z;
            t_target2cam.push_back(temp_tmat);
    }
}



int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);

    rclcpp::NodeOptions node_options;
    geometry_msgs::msg::Pose current_pose;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    move_group_node->get_parameter("hand_eye_cout",hand_eye_cout);
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr   charuco_sub=move_group_node->create_subscription<geometry_msgs::msg::Pose>("/charuco_pose",1,std::bind(&charuco_pose_callback,std::placeholders::_1));         
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node,PLANNING_GROUP);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("lebai_tool0");
    move_group.setGoalPositionTolerance(0.001);//设置末端到达的最大误差容忍
    move_group.setGoalOrientationTolerance(0.01);
    std::string end_link=move_group.getEndEffectorLink();
    move_group.setPlanningTime(30);
    move_group.setPlannerId("TRRT");
    cv::Mat R_cam2gripper=cv::Mat(3,3,CV_64FC1);  
    cv::Mat t_cam2gripper=cv::Mat(3,1,CV_64FC1);
    //机械臂末端位姿范围限制

    int limit[6][2]={55,57,   
                    -1,1,
                    28,29,
                    -1000,1000,
                    -300,300,
                    -900,900};
    float data_cout[hand_eye_cout][6];
    for(int i=0;i<hand_eye_cout;i++)    
    {     
            for(int j=0;j<6;j++)
            {
                data_cout[i][j]=(rand()%(limit[j][1]-limit[j][0]+1)+limit[j][0])/100.0;
            }
    }

    geometry_msgs::msg::Pose temp_p;
    move_group.setNamedTarget("look");//移动到look点
    move_group.move();
    for(int i=0;i<hand_eye_cout;i++)
   {
        RCLCPP_INFO(move_group_node->get_logger(),"\033[1;32m hand eye count:%d  \033[0m  \n",i);

        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion myQuaternion;

        myQuaternion.setRPY(data_cout[i][3]/57.29578,data_cout[i][4]/57.29578,data_cout[i][5]/57.29578);
        target_pose.orientation.x=myQuaternion.getX();
        target_pose.orientation.y=myQuaternion.getY();
        target_pose.orientation.z=myQuaternion.getZ();
        target_pose.orientation.w=myQuaternion.getW();
        target_pose.position.x=data_cout[i][0];
        target_pose.position.y=data_cout[i][1];
        target_pose.position.z=data_cout[i][2];
        move_group.setPoseTarget(target_pose);
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) move_group.execute(my_plan);
        sleep(1);
        cv::Mat temp1;
        geometry_msgs::msg::Pose temp_p=move_group.getCurrentPose(end_link).pose;
        Eigen::Quaterniond q1(temp_p.orientation.w,temp_p.orientation.x,temp_p.orientation.y,temp_p.orientation.z);
        cv::eigen2cv(q1.toRotationMatrix(),temp1);
        R_gripper2base.push_back(temp1);
        cv::Mat temp_tmat(3,1,CV_64FC1);

        temp_tmat.at<double>(0,0)=temp_p.position.x;
        temp_tmat.at<double>(1,0)=temp_p.position.y;
        temp_tmat.at<double>(2,0)=temp_p.position.z;
        t_gripper2base.push_back(temp_tmat);
        move_flag=1;
        sleep(1);
   }
    cv::calibrateHandEye(R_gripper2base,
                         t_gripper2base,
                         R_target2cam,
                         t_target2cam,
                         R_cam2gripper,
                         t_cam2gripper,
                         cv::CALIB_HAND_EYE_TSAI);
    tf2::Matrix3x3 rotation_matrix(R_cam2gripper.at<double>(0,0),R_cam2gripper.at<double>(0,1),R_cam2gripper.at<double>(0,2),
                                   R_cam2gripper.at<double>(1,0),R_cam2gripper.at<double>(1,1),R_cam2gripper.at<double>(1,2),
                                   R_cam2gripper.at<double>(2,0),R_cam2gripper.at<double>(2,1),R_cam2gripper.at<double>(2,2));

    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);
    move_group.setNamedTarget("look");
    move_group.move();
    RCLCPP_INFO(move_group_node->get_logger(),"\033[1;33mhand eye Result: \"%f\", \"%f\", \"%f\" ,\"%f\" ,\"%f\" ,\"%f\" ,\"%f\"  please copy to lm3.launch.py \033[0m",t_cam2gripper.at<double>(0,0),t_cam2gripper.at<double>(1,0),t_cam2gripper.at<double>(2,0),quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());

    rclcpp::shutdown();
    return 0;
}
