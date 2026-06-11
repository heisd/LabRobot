"""Launch the Wheeltec dashboard: rosbridge_websocket + static web server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    http_port = LaunchConfiguration('http_port')
    ws_port = LaunchConfiguration('ws_port')
    video_port = LaunchConfiguration('video_port')
    address = LaunchConfiguration('address')
    enable_video = LaunchConfiguration('enable_video')

    return LaunchDescription([
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='TCP port for the dashboard HTTP server'),
        DeclareLaunchArgument(
            'ws_port', default_value='9090',
            description='TCP port for rosbridge_websocket'),
        DeclareLaunchArgument(
            'video_port', default_value='8081',
            description='TCP port for web_video_server (MJPEG streams)'),
        DeclareLaunchArgument(
            'address', default_value='0.0.0.0',
            description='Interface address to bind the HTTP server to'),
        DeclareLaunchArgument(
            'enable_video', default_value='true',
            description='Start web_video_server alongside the dashboard'),
        DeclareLaunchArgument(
            'enable_watchdog', default_value='true',
            description='Start sensor_watchdog (相机/雷达/IMU/下位机 在线监控 + 串口设备表)'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': ws_port}],
        ),
        Node(
            package='wheeltec_dashboard',
            executable='web_server',
            name='wheeltec_dashboard_web_server',
            output='screen',
            parameters=[{
                'port': http_port,
                'address': address,
            }],
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            condition=IfCondition(enable_video),
            parameters=[{
                'port': video_port,
                'address': address,
            }],
        ),
        # 传感器在线监控: 上/掉线日志进 /rosout, 在线状态+串口设备表发
        # sensor_watchdog/status 供面板"传感器与串口设备"卡渲染。
        Node(
            package='wheeltec_dashboard',
            executable='sensor_watchdog',
            name='sensor_watchdog',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_watchdog')),
        ),
    ])
