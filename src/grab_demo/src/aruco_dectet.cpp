#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 使用OpenCV 4.10的本地头文件（通过绝对路径）
#include "/usr/local/opencv410/include/opencv4/opencv2/objdetect.hpp"
#include "/usr/local/opencv410/include/opencv4/opencv2/calib3d.hpp"  // 用于solvePnP
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
class aruco_dectet:public rclcpp::Node
{
    private:
        int dictionary_id;
        double markerLength;//标签边长
        std::string rgb_result_pub_topic;//识别结果图像话题名
        std::string rgb_image_topic;//订阅话题名
        std::string rgb_image_info;//订阅相机内参话题
        std::string ref_frame;//参考坐标
        std::string obj_frame;//物体坐标
        // OpenCV 4.10中aruco相关类直接使用，不需要makePtr
        cv::aruco::Dictionary dictionary;
        cv::aruco::DetectorParameters detectorParams;
        cv::Ptr<cv::aruco::ArucoDetector> arucoDetector;
        cv::Mat camera_matrix;
        cv::Mat camera_dis;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
        std::shared_ptr<image_transport::ImageTransport>  image_transport_ptr_sub;
        std::shared_ptr<image_transport::ImageTransport>  image_transport_result_ptr_pub;
        image_transport::Subscriber image_sub;
        image_transport::Publisher image_result_pub;
        sensor_msgs::msg::Image::SharedPtr result_img;
      
        bool camera_info=0;
        geometry_msgs::msg::Pose charuco_pose_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;

    private:
        // 添加回调函数的声明
        void rgb_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);

    public:
        void start_devtet();
        aruco_dectet():Node("aruco_node")
        {
            tf_pub=std::make_shared<tf2_ros::TransformBroadcaster>(this);

            this->declare_parameter<float>("markerLength",0.05);
            // 订阅话题 camera->camera_arm
            this->declare_parameter<std::string>("rgb_image_topic","/camera_arm/color/image_raw");
            this->declare_parameter<std::string>("rgb_image_info","/gemini_info");
            this->declare_parameter<std::string>("dectet_result_image_pub","/aruco_dect_image");
            this->declare_parameter<int>("dictionary_id",6);
            this->declare_parameter<std::string>("reference_frame","/camera_link");
            this->declare_parameter<std::string>("object_frame","/aruco_link");



            this->get_parameter("markerLength",markerLength);
            this->get_parameter("rgb_image_topic",rgb_image_topic);
            this->get_parameter("rgb_image_info",rgb_image_info);
            this->get_parameter("dectet_result_image_pub",rgb_result_pub_topic);
            this->get_parameter("dictionary_id",dictionary_id);
            this->get_parameter("reference_frame",ref_frame);
            this->get_parameter("object_frame",obj_frame);


            // 映射dictionary_id到OpenCV枚举值 (6通常对应DICT_6X6_250) @liquanyan 
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
            // OpenCV 4.10中使用ArucoDetector类
            arucoDetector = cv::makePtr<cv::aruco::ArucoDetector>(dictionary, detectorParams);
          
            camera_info_sub=create_subscription<sensor_msgs::msg::CameraInfo>(rgb_image_info,1,std::bind(&aruco_dectet::rgb_info_callback,this,std::placeholders::_1));
            
        }
};



int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto charuco_detector=std::make_shared<aruco_dectet>();
    charuco_detector->start_devtet();
    rclcpp::spin(charuco_detector);
    rclcpp::shutdown();
    return 0;
}

void aruco_dectet::rgb_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    std::vector<int> marker_ids,aruco_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners,rejected_markers;
    cv::Mat image_rgb;
    cv::Mat rotation_matrix,pos_matrix;
    if(camera_info)
    {
        image_rgb=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8)->image;
        // OpenCV 4.10中使用ArucoDetector::detectMarkers
        arucoDetector->detectMarkers(image_rgb, marker_corners, aruco_ids);
		if (!aruco_ids.empty())//如果检测到了aruco标签
        {
            //获取位姿 - 使用OpenCV 4.10的API，需要手动计算或使用solvePnP
            // 对于单个marker，使用solvePnP
            if (marker_corners.size() > 0 && marker_corners[0].size() == 4) {
                std::vector<cv::Point3f> objectPoints;
                objectPoints.push_back(cv::Point3f(-markerLength/2, markerLength/2, 0));
                objectPoints.push_back(cv::Point3f(markerLength/2, markerLength/2, 0));
                objectPoints.push_back(cv::Point3f(markerLength/2, -markerLength/2, 0));
                objectPoints.push_back(cv::Point3f(-markerLength/2, -markerLength/2, 0));
                cv::Mat rvec, tvec;
                cv::solvePnP(objectPoints, marker_corners[0], camera_matrix, camera_dis, rvec, tvec);
                cv::Rodrigues(rvec, rotation_matrix);
                pos_matrix = tvec.t();
            }
            if(marker_corners.size()>0)
            {
                            //绘制标签的边
                            cv::line(image_rgb,marker_corners.at(0).at(0),marker_corners.at(0).at(1),cv::Scalar(0,255,0),2,cv::LINE_AA) ;
                            cv::line(image_rgb,marker_corners.at(0).at(1),marker_corners.at(0).at(2),cv::Scalar(0,255,0),2,cv::LINE_AA) ;
                            cv::line(image_rgb,marker_corners.at(0).at(2),marker_corners.at(0).at(3),cv::Scalar(0,255,0),2,cv::LINE_AA) ;
                            cv::line(image_rgb,marker_corners.at(0).at(3),marker_corners.at(0).at(0),cv::Scalar(0,255,0),2,cv::LINE_AA) ;
                            geometry_msgs::msg::TransformStamped obg_msg;
                            obg_msg.transform.translation.x=pos_matrix.at<double>(0,0);
                            obg_msg.transform.translation.y=pos_matrix.at<double>(0,1);
                            obg_msg.transform.translation.z=pos_matrix.at<double>(0,2);
                          
                            obg_msg.header.stamp=this->now();
                            obg_msg.header.frame_id=ref_frame;
                            obg_msg.child_frame_id=obj_frame;
                            tf_pub->sendTransform(obg_msg);//把标签的位姿信息以动态tf坐标发布
                          
            }           
             
        }


        std_msgs::msg::Header header;
        header.stamp=this->get_clock()->now();
        result_img=cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,image_rgb).toImageMsg();
        image_result_pub.publish(result_img);
        // cv::imshow("image_gray",image_rgb);
        // cv::waitKey(3);    
    }
}

void aruco_dectet::rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
{
    bool K_valid=0;
    bool D_valid=0;
    if(!camera_info)
    {
        //判断内参矩阵是否全为0，有一个不为0认定为有效
        for(uint8_t i=0;i<msg->k.size();i++)
        {
            if(msg->k.at(i)!=0)
            {

                K_valid=1;
                break;
            }
        }
        for(uint8_t i=0;i<msg->d.size();i++)
        {
            if(msg->d.at(i)!=0)
            {

                D_valid=1;
                break;
            }
        }
        if(K_valid && D_valid)//如果相机内参矩阵以及畸变系数有效
        {
            camera_matrix=cv::Mat::zeros(3,3,CV_64F);
            camera_dis=cv::Mat::zeros(1,5,CV_64F);
            for(uint8_t i=0;i<3;i++)
            {
                for(uint8_t j=0;j<3;j++)
                {
                    camera_matrix.at<double>(i,j)=msg->k[i*3+j];
                }
            }
            for(uint8_t i=0;i<5;i++)
            {
                camera_dis.at<double>(0,i)=msg->d[i];
            }
            camera_info=1;
        }
    }
    

}
void aruco_dectet::start_devtet()
{
    image_transport_ptr_sub=std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_sub=image_transport_ptr_sub->subscribe(rgb_image_topic,10,std::bind(&aruco_dectet::rgb_image_callback,this,std::placeholders::_1));
    image_transport_result_ptr_pub=std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_result_pub=image_transport_result_ptr_pub->advertise(rgb_result_pub_topic,1);

}
