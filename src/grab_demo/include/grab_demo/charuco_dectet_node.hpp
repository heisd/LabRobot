#ifndef CHARUCO_DECTET_NODE_HPP
#define CHARUCO_DECTET_NODE_HPP
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv4/opencv2/opencv.hpp>
// OpenCV 4.10中charuco已整合到objdetect模块，不需要单独包含
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
class charuco_dectet:public rclcpp::Node
{
    private:
        std::string ref_frame; //参考坐标
        std::string obj_frame;//目标坐标
        int squaresX;//Charuco板的列数
        int squaresY;//Charuco板的行数
        int dictionary_id;//ArUco字典的ID
        double squareLength;//棋盘格边长
        double markerLength;//aurco码边长
        std::string rgb_result_pub_topic;//识别结果图像话题
        std::string rgb_image_topic;//订阅的rgb话题名   
        std::string rgb_image_info;//订阅的图像内参
        std::string charuco_pose_topic_name;//要发布的charuco标定板坐标话题名
        cv::aruco::Dictionary dictionary; // ArUco字典
        cv::Ptr<cv::aruco::CharucoBoard> charuco_board; // Charuco板指针
        cv::aruco::DetectorParameters detectorParams; // ArUco检测器参数
        cv::Ptr<cv::aruco::ArucoDetector> arucoDetector; // Aruco检测器
        cv::Ptr<cv::aruco::CharucoDetector> charucoDetector; // Charuco检测器
        cv::Mat camera_matrix;//相机内参矩阵
        cv::Mat camera_dis;//相机畸变参数
        cv::Vec3d camera_rotation; // 相机旋转向量
        cv::Vec3d camera_translation;// 相机平移向量
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr charuco_pose_pub; // Charuco位姿发布器
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub; // 相机内参订阅器
        std::shared_ptr<image_transport::ImageTransport>  image_transport_ptr_sub;// 图像传输对象指针（用于订阅）
        std::shared_ptr<image_transport::ImageTransport>  image_transport_result_ptr_pub;// 图像传输对象指针（用于发布）
        image_transport::Subscriber image_sub;  // 图像订阅器
        image_transport::Publisher image_result_pub;// 图像发布器
        cv::Mat temp;
        void rgb_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);
        void draw_axis(cv::InputOutputArray image,cv::InputArray camera_K,cv::InputArray camera_D,cv::InputArray rotation,cv::InputArray translation,float length);
        cv::Mat vec2matrix(cv::Vec3d &camera_rotation,cv::Vec3d &camera_translation); // 相机旋转向量  

        bool camera_info=0;
        geometry_msgs::msg::Pose charuco_pose_;
        sensor_msgs::msg::Image::SharedPtr result_img;
        cv::Mat cam_color2camera_link=(cv::Mat_<double>(4,4) <<
            -0.003 , 0.000 , 1.000 ,-0.001,
            -1.000 , 0.006, -0.003 , 0.010,
            -0.006 ,-1.000 , 0.000, -0.000,
            0.000 , 0.000 , 0.000 , 1.000
        );

    public:
        void start_devtet();
        charuco_dectet():Node("charuco_node")
        {
            this->declare_parameter<int>("squaresX",5);
            this->declare_parameter<int>("squaresY",7);
            this->declare_parameter<float>("squareLength",0.028);
            this->declare_parameter<float>("markerLength",0.0176);
            this->declare_parameter<std::string>("rgb_image_topic","/camera/color/image_raw");
            this->declare_parameter<std::string>("rgb_image_info","/gemini_info");
            this->declare_parameter<std::string>("rgb_result_pub","/charuco_dect");
            this->declare_parameter<int>("dictionary_id",6);
            this->declare_parameter<std::string>("charuco_pose_topic_name","/charuco_pose");


            this->get_parameter("squaresX",squaresX);
            this->get_parameter("squaresY",squaresY);
            this->get_parameter("squareLength",squareLength);
            this->get_parameter("markerLength",markerLength);
            this->get_parameter("rgb_image_topic",rgb_image_topic);
            this->get_parameter("rgb_image_info",rgb_image_info);
            this->get_parameter("rgb_result_pub",rgb_result_pub_topic);
            this->get_parameter("dictionary_id",dictionary_id);
            this->get_parameter("charuco_pose_topic_name",charuco_pose_topic_name);
        
            // 映射dictionary_id到OpenCV枚举值 (6通常对应DICT_6X6_250)
            cv::aruco::PredefinedDictionaryType dict_type;
            switch(dictionary_id) {
                case 0: dict_type = cv::aruco::DICT_4X4_50; break;
                case 1: dict_type = cv::aruco::DICT_4X4_100; break;
                case 2: dict_type = cv::aruco::DICT_4X4_250; break;
                case 3: dict_type = cv::aruco::DICT_4X4_1000; break;
                case 4: dict_type = cv::aruco::DICT_5X5_50; break;
                case 5: dict_type = cv::aruco::DICT_5X5_100; break;
                case 6: dict_type = cv::aruco::DICT_5X5_250; break;
                case 7: dict_type = cv::aruco::DICT_5X5_1000; break;
                case 8: dict_type = cv::aruco::DICT_6X6_50; break;
                case 9: dict_type = cv::aruco::DICT_6X6_100; break;
                case 10: dict_type = cv::aruco::DICT_6X6_250; break;
                case 11: dict_type = cv::aruco::DICT_6X6_1000; break;
                case 12: dict_type = cv::aruco::DICT_7X7_50; break;
                case 13: dict_type = cv::aruco::DICT_7X7_100; break;
                case 14: dict_type = cv::aruco::DICT_7X7_250; break;
                case 15: dict_type = cv::aruco::DICT_7X7_1000; break;
                case 16: dict_type = cv::aruco::DICT_ARUCO_ORIGINAL; break;
                default: dict_type = cv::aruco::DICT_6X6_250; break;
            }
            dictionary = cv::aruco::getPredefinedDictionary(dict_type);
            // OpenCV 4.10中CharucoBoard使用构造函数
            charuco_board = cv::makePtr<cv::aruco::CharucoBoard>(cv::Size(squaresX, squaresY), squareLength, markerLength, dictionary);
            // OpenCV 4.10中使用ArucoDetector类
            arucoDetector = cv::makePtr<cv::aruco::ArucoDetector>(dictionary, detectorParams);
            // OpenCV 4.10中使用CharucoDetector类
            charucoDetector = cv::makePtr<cv::aruco::CharucoDetector>(*charuco_board, cv::aruco::CharucoParameters(), detectorParams);
          
            camera_info_sub=create_subscription<sensor_msgs::msg::CameraInfo>(rgb_image_info,1,std::bind(&charuco_dectet::rgb_info_callback,this,std::placeholders::_1));
            charuco_pose_pub =create_publisher<geometry_msgs::msg::Pose>(charuco_pose_topic_name,1);  

            cv::Mat board_img;
            charuco_board->generateImage(cv::Size(1080,720),board_img,10,1);
        }
};


#endif