#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor(): Node("image_processor")
  {
    // 声明参数，允许通过 launch 文件配置话题名
    this->declare_parameter<std::string>("rgb_topic", "/camera_arm/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera_arm/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera_arm/color/camera_info");
    this->declare_parameter<std::string>("tf_frame_id", "camera_arm_color_optical_frame");
    this->declare_parameter<std::string>("tf_child_frame_id", "detected_object");

    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    tf_frame_id_ = this->get_parameter("tf_frame_id").as_string();
    tf_child_frame_id_ = this->get_parameter("tf_child_frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "RGB topic: %s", rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", camera_info_topic.c_str());

    // 订阅RGB话题
    rgb_sub_.subscribe(this, rgb_topic);

    // 订阅深度话题（使用对齐到彩色图的深度图）
    depth_sub_.subscribe(this, depth_topic);

    // 创建同步策略
    approxSync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), rgb_sub_, depth_sub_);
    approxSync->registerCallback(std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 从相机实际发布的话题读取内参，而不是使用硬编码值
    camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic, 1, std::bind(&ImageProcessor::rgb_info_callback, this, std::placeholders::_1));

    // 初始化TF广播器
    tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 创建窗口和滑动条
    cv::namedWindow("RGB Image");
    cv::namedWindow("Debug Info");

    cv::createTrackbar("Hue Min", "RGB Image", &hue_min_, 255);
    cv::createTrackbar("Hue Max", "RGB Image", &hue_max_, 255);
    cv::createTrackbar("Saturation Min", "RGB Image", &saturation_min_, 255);
    cv::createTrackbar("Saturation Max", "RGB Image", &saturation_max_, 255);
    cv::createTrackbar("Value Min", "RGB Image", &value_min_, 255);
    cv::createTrackbar("Value Max", "RGB Image", &value_max_, 255);
    
    // 添加x方向偏差调整滑动条
    cv::createTrackbar("X Offset (cm)", "RGB Image", &x_offset_cm_, 200);
    cv::setTrackbarMin("X Offset (cm)", "RGB Image", -100);  // -100cm 到 +100cm
    // 添加y方向偏差调整滑动条
    cv::createTrackbar("Y Offset (cm)", "RGB Image", &y_offset_cm_, 200);
    cv::setTrackbarMin("Y Offset (cm)", "RGB Image", -100);  // -100cm 到 +100cm
    // 添加z方向偏差调整滑动条
    cv::createTrackbar("Z Offset (cm)", "RGB Image", &z_offset_cm_, 200);
    cv::setTrackbarMin("Z Offset (cm)", "RGB Image", -100);  // -100cm 到 +100cm
    // 添加调试信息
    debug_info_ = cv::Mat(200, 600, CV_8UC3, cv::Scalar(255, 255, 255));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    if(camera_info)
    {
      // 转换ROS图像消息为OpenCV格式
      cv_bridge::CvImagePtr cv_rgb_ptr;
      try
      {
        cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      cv_bridge::CvImagePtr cv_depth_ptr;
      try
      {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // 检查并记录分辨率，只在分辨率变化时输出一次
      if (cv_rgb_ptr->image.cols != rgb_width_ || cv_rgb_ptr->image.rows != rgb_height_ ||
          cv_depth_ptr->image.cols != depth_width_ || cv_depth_ptr->image.rows != depth_height_)
      {
        rgb_width_ = cv_rgb_ptr->image.cols;
        rgb_height_ = cv_rgb_ptr->image.rows;
        depth_width_ = cv_depth_ptr->image.cols;
        depth_height_ = cv_depth_ptr->image.rows;

        RCLCPP_INFO(this->get_logger(), "RGB image size: %dx%d", rgb_width_, rgb_height_);
        RCLCPP_INFO(this->get_logger(), "Depth image size: %dx%d", depth_width_, depth_height_);

        if (rgb_width_ != depth_width_ || rgb_height_ != depth_height_)
        {
          RCLCPP_WARN(this->get_logger(),
            "WARNING: RGB and Depth image sizes do not match! "
            "Use aligned_depth_to_color topic for accurate 3D positioning.");
        }
      }

      // 图像处理
      cv::Mat hsv_image_raw;
      cv::cvtColor(cv_rgb_ptr->image, hsv_image_raw, cv::COLOR_BGR2HSV);
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
      
      // 设置颜色阈值
      cv::Scalar lower(hue_min_, saturation_min_, value_min_);
      cv::Scalar upper(hue_max_, saturation_max_, value_max_);
      
      // 对HSV图像应用颜色阈值
      cv::Mat threshold_image;
      cv::inRange(hsv_image_raw, lower, upper, threshold_image);
      
      cv::Mat hsv_image_erode, hsv_image_dilate;
      cv::erode(threshold_image, hsv_image_erode, kernel);
      cv::dilate(hsv_image_erode, hsv_image_dilate, kernel);

      // 在原始图像上绘制检测结果的预览
      cv::Mat display_image = cv_rgb_ptr->image.clone();

      // 显示二值化结果（HSV 阈值）
      cv::imshow("HSV Threshold", hsv_image_dilate);

      // 寻找轮廓
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(hsv_image_dilate, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      
      // 寻找面积最大的物体
      std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2)
      {
        return cv::contourArea(c1) > cv::contourArea(c2);
      });
      
      if(!contours.empty())
      {
        std::vector<cv::Point> contour = contours[0];
        cv::Moments moments = cv::moments(contour);

        // 在 RGB 图像上绘制最大轮廓
        cv::drawContours(display_image, contours, 0, cv::Scalar(0, 255, 0), 2);

        // 检查是否除零
        if(moments.m00 > 0)
        {
          cv::Point newpos(moments.m10 / moments.m00, moments.m01 / moments.m00);

          // 在 RGB 图像上绘制中心点
          cv::circle(display_image, newpos, 5, cv::Scalar(0, 0, 255), -1);
          cv::circle(display_image, newpos, 20, cv::Scalar(255, 0, 0), 2);

          // 确保像素坐标在深度图范围内
          int depth_x = newpos.x;
          int depth_y = newpos.y;

          // 如果 RGB 和深度分辨率不同，进行坐标缩放
          if (rgb_width_ != depth_width_ || rgb_height_ != depth_height_)
          {
            depth_x = static_cast<int>(newpos.x * depth_width_ / rgb_width_);
            depth_y = static_cast<int>(newpos.y * depth_height_ / rgb_height_);
          }

          // 边界检查
          depth_x = std::max(0, std::min(depth_x, depth_width_ - 1));
          depth_y = std::max(0, std::min(depth_y, depth_height_ - 1));

          // 获取深度值，使用小区域的中值滤波来提高稳定性
          double dis = 0.0;
          std::vector<ushort> depth_values;
          int window_size = 3;  // 3x3 窗口
          for (int dy = -window_size/2; dy <= window_size/2; dy++)
          {
            for (int dx = -window_size/2; dx <= window_size/2; dx++)
            {
              int px = std::max(0, std::min(depth_x + dx, depth_width_ - 1));
              int py = std::max(0, std::min(depth_y + dy, depth_height_ - 1));
              ushort d = cv_depth_ptr->image.at<ushort>(py, px);
              if (d > 0) depth_values.push_back(d);
            }
          }

          if (!depth_values.empty())
          {
            std::sort(depth_values.begin(), depth_values.end());
            dis = depth_values[depth_values.size() / 2] / 1000.0;  // 取中值
          }

          // 有效的深度范围检查（典型深度相机工作范围：0.2m - 5m）
          if(dis > 0.15 && dis < 5.0)
          {
            // 通过相机内参将像素坐标转换成物体相对相机的三维坐标
            double x = (newpos.x - camera_matrix.at<double>(0,2)) / camera_matrix.at<double>(0,0) * dis;
            double y = (newpos.y - camera_matrix.at<double>(1,2)) / camera_matrix.at<double>(1,1) * dis;

            // 应用x方向偏差调整（转换为米）
            double x_offset = x_offset_cm_ / 100.0;
            x += x_offset;
            double y_offset = y_offset_cm_ / 100.0;
            y += y_offset;
            double z_offset = z_offset_cm_ / 100.0;
            dis += z_offset;

            // 发布TF变换，使用可配置的坐标系名称
            geometry_msgs::msg::TransformStamped obg_msg;
            obg_msg.transform.translation.x = x;
            obg_msg.transform.translation.y = y;
            obg_msg.transform.translation.z = dis;
            obg_msg.header.stamp = this->now();
            obg_msg.header.frame_id = tf_frame_id_;
            obg_msg.child_frame_id = tf_child_frame_id_;
            obg_msg.transform.rotation.x = 0.0;
            obg_msg.transform.rotation.y = 0.0;
            obg_msg.transform.rotation.z = 0.0;
            obg_msg.transform.rotation.w = 1.0;

            tf_pub->sendTransform(obg_msg);
            RCLCPP_INFO(this->get_logger(), "Dis: %.3f, X: %.3f, Y: %.3f", dis, x, y);

            // 更新调试信息
            updateDebugInfo(x, y, dis, newpos, x_offset, y_offset, z_offset);
          }
          else if (dis > 0)
          {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Depth value %.3f out of valid range (0.15-5.0m)", dis);
          }
        }
      }

      // 显示带标记的 RGB 图像（在所有绘制完成后）
      cv::imshow("RGB Image", display_image);
      cv::waitKey(1);
    }
  }

  void updateDebugInfo(double x, double y, double z, cv::Point pixel_pos, double x_offset, double y_offset, double z_offset)
  {
    debug_info_.setTo(cv::Scalar(255, 255, 255));
    
    std::stringstream ss;
    ss << "3D Position:";
    cv::putText(debug_info_, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    ss.str(""); ss << "X: " << std::fixed << std::setprecision(3) << x << " m";
    cv::putText(debug_info_, ss.str(), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    
    ss.str(""); ss << "Y: " << std::fixed << std::setprecision(3) << y << " m";
    cv::putText(debug_info_, ss.str(), cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    
    ss.str(""); ss << "Z: " << std::fixed << std::setprecision(3) << z << " m";
    cv::putText(debug_info_, ss.str(), cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    
    ss.str(""); ss << "Pixel: (" << pixel_pos.x << ", " << pixel_pos.y << ")";
    cv::putText(debug_info_, ss.str(), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);  
    ss.str(""); ss << "X Offset: " << x_offset_cm_ << " cm";
    cv::putText(debug_info_, ss.str(), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    ss.str(""); ss << "Y Offset: " << y_offset_cm_ << " cm";
    cv::putText(debug_info_, ss.str(), cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    ss.str(""); ss << "Z Offset: " << z_offset_cm_ << " cm";
    cv::putText(debug_info_, ss.str(), cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    
    cv::imshow("Debug Info", debug_info_);
  }

  void rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
  {
    bool K_valid = false;
    bool D_valid = false;
    
    if(!camera_info)
    {
      for(uint8_t i = 0; i < msg->k.size(); i++)
      {
        if(msg->k.at(i) != 0)
        {
          K_valid = true;
          break;
        }
      }
      
      for(uint8_t i = 0; i < msg->d.size(); i++)
      {
        if(msg->d.at(i) != 0)
        {
          D_valid = true;
          break;
        }
      }
      
      if(K_valid && D_valid)
      {
        camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        camera_dis = cv::Mat::zeros(1, 5, CV_64F);
        
        for(uint8_t i = 0; i < 3; i++)
        {
          for(uint8_t j = 0; j < 3; j++)
          {
            camera_matrix.at<double>(i, j) = msg->k[i * 3 + j];
          }
        }
        
        for(uint8_t i = 0; i < 5; i++)
        {
          camera_dis.at<double>(0, i) = msg->d[i];
        }
        
        camera_info = true;
        RCLCPP_INFO(this->get_logger(), "Camera calibration loaded successfully");
        
        // 打印相机内参用于调试
        RCLCPP_INFO(this->get_logger(), "Camera Matrix:");
        RCLCPP_INFO(this->get_logger(), "[%f, %f, %f]", 
                   camera_matrix.at<double>(0,0), camera_matrix.at<double>(0,1), camera_matrix.at<double>(0,2));
        RCLCPP_INFO(this->get_logger(), "[%f, %f, %f]", 
                   camera_matrix.at<double>(1,0), camera_matrix.at<double>(1,1), camera_matrix.at<double>(1,2));
        RCLCPP_INFO(this->get_logger(), "[%f, %f, %f]", 
                   camera_matrix.at<double>(2,0), camera_matrix.at<double>(2,1), camera_matrix.at<double>(2,2));
      }
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> syncPolicy;
  message_filters::Synchronizer<syncPolicy> *approxSync;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  cv::Mat camera_matrix;
  cv::Mat camera_dis;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

  // HSV 默认值 - 设置为检测红色物体的范围（可通过滑动条调整）
  // 注意: OpenCV 中 H 范围是 0-179，S 和 V 是 0-255
  int hue_min_ = 0;
  int hue_max_ = 10;  // 红色在 HSV 中 H 值接近 0 或 180
  int saturation_min_ = 100;  // 过滤掉低饱和度的颜色
  int saturation_max_ = 255;
  int value_min_ = 100;  // 过滤掉太暗的区域
  int value_max_ = 255;
  int x_offset_cm_ = 0;  // x方向偏差调整（厘米）
  int y_offset_cm_ = 0;  // y方向偏差调整（厘米）
  int z_offset_cm_ = 0;  // z方向偏差调整（厘米）
  bool camera_info = false;
  cv::Mat debug_info_;

  // TF 坐标系名称（可通过参数配置）
  std::string tf_frame_id_;
  std::string tf_child_frame_id_;

  // 图像分辨率（用于检查对齐）
  int rgb_width_ = 0;
  int rgb_height_ = 0;
  int depth_width_ = 0;
  int depth_height_ = 0;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}