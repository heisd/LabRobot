import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
    )
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    # 巡线节点: 速度输出重映射到中间话题 line_follow/cmd_vel, 交给仲裁器统一裁决
    line_follow_node = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='line_follow',
        name='line_follow',
        remappings=[('cmd_vel', 'line_follow/cmd_vel')],
    )

    # QR 检测节点: 检测到二维码时发布事件(detected / data / area_ratio)
    qr_detector_node = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='qr_detector',
        name='qr_detector',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'min_area_ratio': 0.002,
            'show_image': False,
        }],
    )

    # 速度仲裁器: QR 事件优先级高于巡线, 检测到二维码先减速后停下
    cmd_arbiter_node = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='cmd_arbiter',
        name='cmd_arbiter',
        parameters=[{
            'decel_duration': 1.2,
            'publish_rate': 20.0,
            'detect_timeout': 0.5,
            'clear_hold': 1.0,
            'resume_after_clear': True,
        }],
    )

    return LaunchDescription([
        wheeltec_robot,
        wheeltec_camera,
        line_follow_node,
        qr_detector_node,
        cmd_arbiter_node,
    ])
