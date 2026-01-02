#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class PointCloudPCLVisualizer : public rclcpp::Node
{
public:
    PointCloudPCLVisualizer() : Node("pointcloud_pcl_visualizer")
    {
        // 创建 PCL 可视化器
        viewer_.reset(new pcl::visualization::CloudViewer("Point Cloud Viewer"));
        
        // 订阅点云话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_arm/depth/points",
            10,
            std::bind(&PointCloudPCLVisualizer::pointcloud_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera_arm/depth/points");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // PCL数据在Jetson上不兼容,使用这个方式我们发现我们的显示不正常
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        
        // 显示点云
        viewer_->showCloud(cloud);
    }
    
    boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPCLVisualizer>());
    rclcpp::shutdown();
    return 0;
}