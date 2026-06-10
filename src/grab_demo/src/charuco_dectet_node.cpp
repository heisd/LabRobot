#include "grab_demo/charuco_dectet_node.hpp"
// 使用OpenCV 4.10的本地头文件（通过绝对路径）
#include "/usr/local/opencv410/include/opencv4/opencv2/objdetect.hpp"
#include "/usr/local/opencv410/include/opencv4/opencv2/calib3d.hpp"  // 用于solvePnP


int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto charuco_detector=std::make_shared<charuco_dectet>();
    charuco_detector->start_devtet();
    rclcpp::spin(charuco_detector);
    rclcpp::shutdown();
    return 0;
}

//根据位姿绘制坐标轴
void charuco_dectet::draw_axis(cv::InputOutputArray &image,cv::InputArray camera_K,cv::InputArray camera_D,cv::InputArray rotation,cv::InputArray translation,float length)
{
    std::vector<cv::Point3f> axis_point;
    axis_point.push_back(cv::Point3f(0,0,0));
    axis_point.push_back(cv::Point3f(length,0,0));
    axis_point.push_back(cv::Point3f(0,length,0));
    axis_point.push_back(cv::Point3f(0,0,length));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_point,rotation,translation,camera_K,camera_D,image_points);
    cv::line(image,image_points[0],image_points[1],cv::Scalar(255,0,0),3);
    cv::line(image,image_points[0],image_points[2],cv::Scalar(0,255,0),3);
    cv::line(image,image_points[0],image_points[3],cv::Scalar(0,0,255),3);
}

cv::Mat charuco_dectet::vec2matrix(cv::Vec3d &camera_rotation,cv::Vec3d &camera_translation) // 相机旋转向量  
{
    cv::Mat temp_matrix;
    cv::Rodrigues(camera_rotation,temp_matrix);
    cv::Mat transform_matrix=cv::Mat::eye(4,4,CV_64F);
    for(uint8_t i=0;i<3;i++)
    {
        for(uint8_t j=0;j<3;j++)
        {
            transform_matrix.at<double>(i,j)=temp_matrix.at<double>(i,j);
        }
    }

    for(uint8_t i=0;i<3;i++)
    {
        transform_matrix.at<double>(i,3)=camera_translation[i];
    }
    return transform_matrix;
    

}


void charuco_dectet::rgb_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    std::vector<int> marker_ids,charuco_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners,rejected_markers;
    std::vector<cv::Point2f> charuco_corners;
    cv::Mat image_rgb;
    cv::Scalar color(255,0,0);
    if(camera_info)//如果订阅到有效相机内参
    {
        image_rgb=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8)->image;
        // OpenCV 4.10中使用ArucoDetector::detectMarkers
        std::vector<std::vector<cv::Point2f>> rejected_markers_vec;
        arucoDetector->detectMarkers(image_rgb, marker_corners, marker_ids, rejected_markers_vec);
        rejected_markers = rejected_markers_vec;
        // OpenCV 4.10中使用ArucoDetector::refineDetectedMarkers
        arucoDetector->refineDetectedMarkers(image_rgb, *charuco_board, marker_corners, marker_ids, rejected_markers, camera_matrix, camera_dis);
        
        // OpenCV 4.10中使用CharucoDetector::detectBoard来检测charuco角点
        std::vector<cv::Point2f> charuco_corners_detected;
        std::vector<int> charuco_ids_detected;
        charucoDetector->detectBoard(image_rgb, charuco_corners_detected, charuco_ids_detected, marker_corners, marker_ids);
        charuco_corners = charuco_corners_detected;
        charuco_ids = charuco_ids_detected;

		bool valid_pose = false;
		if (camera_matrix.total() != 0 && !charuco_ids.empty()) {
			// 使用solvePnP进行pose估计（OpenCV 4.10中estimatePoseCharucoBoard可能不可用）
			cv::Mat rvec, tvec;
			if (charuco_corners.size() >= 4) {
				// 获取charuco板的所有3D角点
				std::vector<cv::Point3f> allCharuco3D = charuco_board->getChessboardCorners();
				// 根据检测到的charuco_ids找到对应的3D点
				std::vector<cv::Point3f> charuco3D;
				std::vector<cv::Point2f> charuco2D;
				for (size_t i = 0; i < charuco_ids.size() && i < charuco_corners.size(); i++) {
					int id = charuco_ids[i];
					if (id >= 0 && id < static_cast<int>(allCharuco3D.size())) {
						charuco3D.push_back(allCharuco3D[id]);
						charuco2D.push_back(charuco_corners[i]);
					}
				}
				if (charuco3D.size() >= 4) {
					valid_pose = cv::solvePnP(charuco3D, charuco2D, camera_matrix, camera_dis, rvec, tvec);
					camera_rotation = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
					camera_translation = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
				}
			}
		}
        
        cv::Mat charuco2camera_color=charuco_dectet::vec2matrix(camera_rotation,camera_translation);
        cv::Mat cam2charuco=cam_color2camera_link*charuco2camera_color;
        cv::Mat temp_matrix;
        temp_matrix=cam2charuco(cv::Rect(0,0,3,3)).clone();
        Eigen::Matrix3d eigen_rotation_matrix;
        cv::cv2eigen(temp_matrix,eigen_rotation_matrix);
        Eigen::Quaterniond q(eigen_rotation_matrix);

        charuco_pose_.position.x=cam2charuco.at<double>(0,3);
        charuco_pose_.position.y=cam2charuco.at<double>(1,3);
        charuco_pose_.position.z=cam2charuco.at<double>(2,3);
        
        charuco_pose_.orientation.x=q.x();
        charuco_pose_.orientation.y=q.y();
        charuco_pose_.orientation.z=q.z();
        charuco_pose_.orientation.w=q.w();

        charuco_pose_pub->publish(charuco_pose_);
        cv::aruco::drawDetectedMarkers(image_rgb,marker_corners);
        if(valid_pose)
        {
                     draw_axis(image_rgb,camera_matrix,camera_dis,camera_rotation,camera_translation,0.1);
        }

        for(size_t i=0;i<charuco_ids.size();i++)
        {
            cv::circle(image_rgb,charuco_corners.at(i),3,cv::Scalar(255,0,255),-1);

        }
        std_msgs::msg::Header header;
        header.stamp=this->get_clock()->now();
        result_img=cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,image_rgb).toImageMsg();
        image_result_pub.publish(result_img);
    }
}

void charuco_dectet::rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
{
    bool K_valid=0;
    bool D_valid=0;
    if(!camera_info)
    {

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
        if(K_valid && D_valid)
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

void charuco_dectet::start_devtet()
{
    image_transport_ptr_sub=std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_sub=image_transport_ptr_sub->subscribe(rgb_image_topic,10,std::bind(&charuco_dectet::rgb_image_callback,this,std::placeholders::_1));
    image_transport_result_ptr_pub=std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_result_pub=image_transport_result_ptr_pub->advertise(rgb_result_pub_topic,1);

}