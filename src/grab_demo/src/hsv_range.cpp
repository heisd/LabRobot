
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
    // 订阅RGB话题
    rgb_sub_.subscribe(this, "/camera_arm/color/image_raw");

    mera_info_sub=create_subscription<sensor_msgs::msg::CameraInfo>("/gemini_info",1,std::bind(&ImageProcessor::rgb_info_callback,this,std::placeholders::_1));

    // 创建窗口和滑动条
    // 发现好像是阻塞在这里
    // 显示黑屏来检测是不是这里出现问题
    try{
        cv::namedWindow("RGB Image");
    }  
    catch(cv::Exception& e){
      RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),5000,"无法显示RGB 窗口");
    }
    // 使用回调函数而不是直接传递指针，避免警告
    cv::createTrackbar("Hue Min", "RGB Image", nullptr, 255, onHueMin, this);
    cv::createTrackbar("Hue Max", "RGB Image", nullptr, 255, onHueMax, this);
    cv::createTrackbar("Saturation Min", "RGB Image", nullptr, 255, onSaturationMin, this);
    cv::createTrackbar("Saturation Max", "RGB Image", nullptr, 255, onSaturationMax, this);
    cv::createTrackbar("Value Min", "RGB Image", nullptr, 255, onValueMin, this);
    cv::createTrackbar("Value Max", "RGB Image", nullptr, 255, onValueMax, this);

    // 设置初始值
    cv::setTrackbarPos("Hue Min", "RGB Image", hue_min_);
    cv::setTrackbarPos("Hue Max", "RGB Image", hue_max_);
    cv::setTrackbarPos("Saturation Min", "RGB Image", saturation_min_);
    cv::setTrackbarPos("Saturation Max", "RGB Image", saturation_max_);
    cv::setTrackbarPos("Value Min", "RGB Image", value_min_);
    cv::setTrackbarPos("Value Max", "RGB Image", value_max_);
  }

private:
  // 为每个滑动条创建单独的回调函数
  static void onHueMin(int value, void* userdata)
  {
    ImageProcessor* processor = static_cast<ImageProcessor*>(userdata);
    processor->hue_min_ = value;
  }
  
  static void onHueMax(int value, void* userdata)
  {
    ImageProcessor* processor = static_cast<ImageProcessor*>(userdata);
    processor->hue_max_ = value;
  }
  
  static void onSaturationMin(int value, void* userdata)
  {
    ImageProcessor* processor = static_cast<ImageProcessor*>(userdata);
    processor->saturation_min_ = value;
  }
  
  static void onSaturationMax(int value, void* userdata)
  {
    ImageProcessor* processor = static_cast<ImageProcessor*>(userdata);
    processor->saturation_max_ = value;
  }
  
  static void onValueMin(int value, void* userdata)
  {
    ImageProcessor* processor = static_cast<ImageProcessor*>(userdata);
    processor->value_min_ = value;
  }
  
  static void onValueMax(int value, void* userdata)
  {
    ImageProcessor* processor = static_cast<ImageProcessor*>(userdata);
    processor->value_max_ = value;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {// 订阅深度话题
    depth_sub_.subscribe(this, "/camera_arm/depth/image_raw");

    // 创建同步策略
    approxSync=new message_filters::Synchronizer<syncPolicy>(syncPolicy(10),rgb_sub_,depth_sub_);
    approxSync->registerCallback(std::bind(&ImageProcessor::imageCallback, this,std::placeholders::_1, std::placeholders::_2));
    ca
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

        // 将图像从BGR颜色空间转换到HSV颜色空间
        cv::Mat hsv_image_raw;
        cv::cvtColor(cv_rgb_ptr->image, hsv_image_raw, cv::COLOR_BGR2HSV);
        cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
        cv::Mat hsv_image_erode;
        cv::Mat hsv_image_dilate;
        // 设置颜色阈值
        cv::Scalar lower(hue_min_, saturation_min_, value_min_);
        cv::Scalar upper(hue_max_, saturation_max_, value_max_);
        // 对HSV图像应用颜色阈值
        cv::Mat threshold_image;
        cv::inRange(hsv_image_raw, lower, upper, threshold_image);
        cv::erode(threshold_image,hsv_image_erode,kernel);
        cv::dilate(hsv_image_erode,hsv_image_dilate,kernel);
        // 显示结果图像
        cv::imshow("RGB Image", hsv_image_dilate);
        cv::waitKey(1);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(hsv_image_dilate,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        //寻找面积最大的物体
        std::sort(contours.begin(),contours.end(),[](const std::vector<cv::Point>&c1,const std::vector<cv::Point>& c2)
        {
          return  cv::contourArea(c1) >cv::contourArea(c2);
        });
        // 如果图像不为空
        if(!contours.empty())
        {
          std::vector<cv::Point> contour=contours[0];
          cv::Moments moments=cv::moments(contour);
          cv::Point newpos(moments.m10/moments.m00,moments.m01/moments.m00);
          // 在RGB图像上对识别到的物体做标记
          // 让我们方便知道我们定位抓取的物体
          cv::Mat display_image = cv_rgb_ptr->image.clone();
          int cross_size = 20;  // 十字大小
          int thickness = 2;    // 线条粗细
          cv::Scalar blue(255, 0, 0);  // BGR格式的蓝色
          
          // 画水平线
          cv::line(display_image, 
                  cv::Point(newpos.x - cross_size, newpos.y),
                  cv::Point(newpos.x + cross_size, newpos.y),
                  blue, thickness);
          // 画垂直线
          cv::line(display_image,
                  cv::Point(newpos.x, newpos.y - cross_size),
                  cv::Point(newpos.x, newpos.y + cross_size),
                  blue, thickness);
          
          // 可选：画轮廓边框
          cv::drawContours(display_image, contours, 0, cv::Scalar(0, 255, 0), 2);
          
          // 显示带标记的图像
          cv::imshow("Detection", display_image);
          cv::waitKey(1);
          // 创建TF广播器
          tf_pub=std::make_shared<tf2_ros::TransformBroadcaster>(this);
          // 深度图像已经处理好我们的距离，我们只是找这个点的位置来判断这个点的深度值
          dis=(cv_depth_ptr->image.at<ushort>(newpos.y,newpos.x))/1000.0;
          double x=(newpos.x-camera_matrix.at<double>(0,2))/camera_matrix.at<double>(0,0)*dis;//通过相机内参将像素坐标转换成物体相对相机的三维坐标
          double y=(newpos.y-camera_matrix.at<double>(1,2))/camera_matrix.at<double>(1,1)*dis;
          RCLCPP_INFO(this->get_logger(),"dis = %.3f",dis);
          if(dis>0)
          {
              // 在运行的时候添加Z轴补偿+0.07m的补偿，这个要根据实际情况进行更改
              double z_offset =0.07;
              geometry_msgs::msg::TransformStamped obg_msg;
              obg_msg.transform.translation.x=x;
              obg_msg.transform.translation.y=y;
              obg_msg.transform.translation.z=dis+z_offset;
              obg_msg.header.stamp=this->now();
              // 修改为 camera_link,机器臂可以移动,但是位置不对
              // 在返回到camera_arm_depth_optical_frame
              obg_msg.header.frame_id="camera_arm_depth_optical_frame";    //设置参考坐标
              obg_msg.child_frame_id="target_frame";    //物体的tf坐标
              // 发布camera_arm_depth_optical_frame到target_frame的坐标转换，我们手动指定
              tf_pub->sendTransform(obg_msg);
          }      
        }
    }
  }
// 验证相机内参和畸变系数数据是否有效，如果有效的话我们存在这两个数组里面
// camera_martix 相机内参矩阵
// camera_dis 畸变系数
void rgb_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
{
    // 相机内参矩阵
    bool K_valid=0;
    // 畸变系数
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
// 系统的同步通信使用
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image> syncPolicy;
        message_filters::Synchronizer<syncPolicy> *approxSync;
        // 图片信息，深度和彩色图片
        message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
        // 在rgb_info_callback里面初始化
        cv::Mat camera_matrix;
        cv::Mat camera_dis;
        // 调用节点创建服务
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
  // 滑动条参数
  int hue_min_ = 0;
  int hue_max_ = 255;
  int saturation_min_ = 0;
  int saturation_max_ = 255;
  int value_min_ = 0;
  int value_max_ = 255;
  // 获取相机
  bool camera_info=0;
  // 获取深度的距离
  volatile double dis=0;
  // tf 广播器
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