#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
// 使用pclk库，将ROS消息转化成pcl消息，发现不可以使用在Jetson平台上
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
class PointCloudNode : public rclcpp::Node
{
public:
    PointCloudNode() : Node("point_cloud_node")
    {
        // 声明参数，点云需要的参数有：点云话题、点云坐标系、点云队列大小
        this->declare_parameter("point_cloud_topic", "/camera_arm/depth/points");
        this->declare_parameter("point_cloud_frame", "point_cloud_frame");
        this->declare_parameter("point_cloud_queue_size", 10);
        // 获取参数，从launch文件中获取
        std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
        std::string point_cloud_frame = this->get_parameter("point_cloud_frame").as_string();
        int point_cloud_queue_size = this->get_parameter("point_cloud_queue_size").as_int();
        RCLCPP_INFO(this->get_logger(), "PointCloudNode has been started.");
        // 打印参数
        RCLCPP_INFO(this->get_logger(), "point_cloud_topic: %s", point_cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "point_cloud_frame: %s", point_cloud_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "point_cloud_queue_size: %d", point_cloud_queue_size);
        // 订阅点云话题
       subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic, point_cloud_queue_size, std::bind(&PointCloudNode::pointCloudCallback, this, std::placeholders::_1));
        // 创建空窗口，配和cv::imshow显示点云
        cv::namedWindow("Point Cloud", cv::WINDOW_AUTOSIZE);
        
    }
// 回调函数，将ROS消息转换为OpenCV消息
void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{   
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %d x %d points", 
                   msg->width, msg->height);
        
        // 将PointCloud2转换为OpenCV Mat的几种方法
        
        // 方法1: 提取深度信息并创建灰度图像
        cv::Mat depth_image = extractDepthFromPointCloud2(msg);
        
        if (!depth_image.empty()) {
            cv::imshow("Point Cloud Depth Image", depth_image);
            cv::waitKey(1);
        }
        
        // 方法2: 提取RGB信息（如果点云包含RGB字段）
        cv::Mat rgb_image = extractRGBFromPointCloud2(msg);
        if (!rgb_image.empty()) {
            cv::imshow("Point Cloud RGB Image", rgb_image);
            cv::waitKey(1);
        }

}
// 从点云里面获取深度信息,并将其归一化到0-255
cv::Mat extractDepthFromPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    int z_idx = -1;
    for (size_t i = 0; i < msg->fields.size(); ++i) {
        if (msg->fields[i].name == "z") {
            z_idx = i;
            break;
        }
    }
    
    if (z_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No z field found");
        return cv::Mat();
    }
    
    cv::Mat depth_image(msg->height, msg->width, CV_32F);
    
    // 收集有效深度值
    std::vector<float> valid_depths;
    valid_depths.reserve(msg->height * msg->width);
    
    for (uint32_t v = 0; v < msg->height; ++v) {
        for (uint32_t u = 0; u < msg->width; ++u) {
            uint32_t index = v * msg->width + u;
            float z_value = *reinterpret_cast<const float*>(
                &msg->data[index * msg->point_step + msg->fields[z_idx].offset]);
            
            depth_image.at<float>(v, u) = z_value;
            
            if (std::isfinite(z_value) && z_value > 0.1 && z_value < 10.0) {
                valid_depths.push_back(z_value);
            }
        }
    }
    
    if (valid_depths.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid depth values");
        return cv::Mat();
    }
    
    // 用百分位数确定显示范围，排除极端值
    std::sort(valid_depths.begin(), valid_depths.end());
    float min_display = valid_depths[valid_depths.size() * 0.02];  // 2% 百分位
    float max_display = valid_depths[valid_depths.size() * 0.98];  // 98% 百分位
    
    RCLCPP_INFO(this->get_logger(), 
        "Display range: %.3f - %.3f m", min_display, max_display);
    
    // 归一化到显示范围
    cv::Mat display_image(msg->height, msg->width, CV_8UC1);
    float range = max_display - min_display;
    if (range < 0.01) range = 0.01;  // 防止除零
    
    for (uint32_t v = 0; v < msg->height; ++v) {
        for (uint32_t u = 0; u < msg->width; ++u) {
            float z = depth_image.at<float>(v, u);
            uint8_t pixel_val;
            
            if (!std::isfinite(z)) {
                pixel_val = 0;  // 无效点设为黑色
            } else {
                // 裁剪到范围内并归一化
                z = std::max(min_display, std::min(max_display, z));
                pixel_val = static_cast<uint8_t>(255.0 * (z - min_display) / range);
            }
            display_image.at<uint8_t>(v, u) = pixel_val;
        }
    }
    
    // 伪彩色,将灰度图像display_image 转换成伪彩色图像colored
    cv::Mat colored;
    cv::applyColorMap(display_image, colored, cv::COLORMAP_JET);
    
    return colored;
}
  // 从PointCloud2提取RGB信息
   cv::Mat extractRGBFromPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 查找 rgb 或 rgba 字段
    int rgb_idx = -1;
    std::string field_name;
    for (size_t i = 0; i < msg->fields.size(); ++i) {
        if (msg->fields[i].name == "rgb" || msg->fields[i].name == "rgba") {
            rgb_idx = i;
            field_name = msg->fields[i].name;
            break;
        }
    }
    
    if (rgb_idx == -1) {
        // 打印所有可用字段，帮助调试
        std::string available_fields;
        for (const auto& field : msg->fields) {
            available_fields += field.name + " ";
        }
        RCLCPP_WARN(this->get_logger(), 
            "No rgb/rgba field found. Available fields: %s", available_fields.c_str());
        return cv::Mat();
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "Found '%s' field at index %d, offset %d", 
        field_name.c_str(), rgb_idx, msg->fields[rgb_idx].offset);
    
    cv::Mat rgb_image(msg->height, msg->width, CV_8UC3);
    
    // 统计信息
    int valid_count = 0;
    int black_count = 0;
    uint32_t sample_rgb = 0;
    
    for (uint32_t v = 0; v < msg->height; ++v) {
        for (uint32_t u = 0; u < msg->width; ++u) {
            uint32_t index = v * msg->width + u;
            uint32_t rgb_value = *reinterpret_cast<const uint32_t*>(
                &msg->data[index * msg->point_step + msg->fields[rgb_idx].offset]);
            
            uint8_t r = (rgb_value >> 16) & 0xFF;
            uint8_t g = (rgb_value >> 8) & 0xFF;
            uint8_t b = rgb_value & 0xFF;
            
            cv::Vec3b& pixel = rgb_image.at<cv::Vec3b>(v, u);
            pixel[2] = r;  // OpenCV 是 BGR 顺序
            pixel[1] = g;
            pixel[0] = b;
            
            // 统计
            if (r == 0 && g == 0 && b == 0) {
                black_count++;
            } else {
                valid_count++;
                if (valid_count == 1) {
                    sample_rgb = rgb_value;  // 记录第一个非黑像素
                }
            }
        }
    }
    
    // 打印调试信息
    RCLCPP_INFO(this->get_logger(), 
        "RGB stats: valid=%d, black=%d, total=%d", 
        valid_count, black_count, msg->height * msg->width);
    
    if (valid_count > 0) {
        RCLCPP_INFO(this->get_logger(), 
            "Sample RGB value: 0x%08X -> R=%d G=%d B=%d",
            sample_rgb,
            (sample_rgb >> 16) & 0xFF,
            (sample_rgb >> 8) & 0xFF,
            sample_rgb & 0xFF);
    }
    
    // 如果大部分是黑的，可能是数据格式问题
    float black_ratio = (float)black_count / (msg->height * msg->width);
    if (black_ratio > 0.9) {
        RCLCPP_WARN(this->get_logger(), 
            "%.1f%% pixels are black - possible data format issue!", black_ratio * 100);
        
        // 尝试用 float 解析（有些相机用 float 存 RGB）
        RCLCPP_INFO(this->get_logger(), "Trying float interpretation...");
        float float_rgb = *reinterpret_cast<const float*>(
            &msg->data[msg->fields[rgb_idx].offset]);
        RCLCPP_INFO(this->get_logger(), "First pixel as float: %f", float_rgb);
    }
    
    return rgb_image;
}
private:
    // 创建订阅者，订阅点云消息
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudNode>());
    rclcpp::shutdown();
    return 0;
}