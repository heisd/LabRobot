"""Launch the Wheeltec dashboard: rosbridge_websocket + static web server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    http_port = LaunchConfiguration('http_port')
    ws_port = LaunchConfiguration('ws_port')
    address = LaunchConfiguration('address')

    return LaunchDescription([
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='TCP port for the dashboard HTTP server'),
        DeclareLaunchArgument(
            'ws_port', default_value='9090',
            description='TCP port for rosbridge_websocket'),
        DeclareLaunchArgument(
            'address', default_value='0.0.0.0',
            description='Interface address to bind the HTTP server to'),

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
    ])
