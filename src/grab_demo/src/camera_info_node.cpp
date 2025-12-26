#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
using namespace std::chrono_literals;

//通过相机内参标定获取相机内参矩阵以及畸变系数
std::array<double,9> camera_matrix={466.197706405427, 0.0, 333.31741909299393, 
                                    0.0, 466.7987230714805, 274.33063394222324, 
                                    0.0, 0.0, 1.0};

std::vector<double> dis{0.13958700332889346, -0.13816116524561173, 0.017677123626179035, 0.008839633769697913, 0.0}; 








class camera_info: public rclcpp::Node
{
    public:
        camera_info():Node("camera_info_pub")
        {
            info_pub=create_publisher<sensor_msgs::msg::CameraInfo>("/gemini_info",1);
            time=create_wall_timer(50ms,std::bind(&camera_info::time_callback,this));
        }
    private:
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
        rclcpp::TimerBase::SharedPtr time;
        void time_callback()//以50ms的频率发布相机内参话题
        {
            auto message=sensor_msgs::msg::CameraInfo();
            message.k=camera_matrix;
            message.d=dis;
            info_pub->publish(message);
        }


};
int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto camera_info_node=std::make_shared<camera_info>();
    rclcpp::spin(camera_info_node);
    rclcpp::shutdown();
    return 0;
}
