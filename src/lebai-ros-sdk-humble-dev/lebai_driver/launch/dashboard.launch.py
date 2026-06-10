from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 机械臂 Web Dashboard。
    # 只负责界面与服务调用, 不直接连机器人, 所以无需 robot_ip。
    # 它依赖 robot_state / io_service / system_service / motion 这些节点已经启动:
    #   ros2 launch lebai_driver robot_state.launch.py
    #   ros2 launch lebai_driver io_service.launch.py
    #   ros2 launch lebai_driver system_service.launch.py
    #   ros2 launch lebai_driver motion.launch.py
    # 然后启动本 Dashboard, 浏览器访问 http://<Jetson-IP>:8080/
    http_host = LaunchConfiguration('http_host')
    http_port = LaunchConfiguration('http_port')

    return LaunchDescription([
        DeclareLaunchArgument('http_host', default_value='0.0.0.0',
                              description='HTTP 监听地址, 0.0.0.0 允许局域网访问'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='Dashboard 网页端口'),
        Node(
            package='lebai_driver',
            executable='dashboard',
            name='dashboard_node',
            output='screen',
            parameters=[{
                'http_host': http_host,
                'http_port': http_port,
            }],
        ),
    ])
