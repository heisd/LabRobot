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

    # QR 检测节点: 抽帧 + 缩放 + 连续确认, 只有"确认到二维码"才让仲裁器停车;
    # 仅仅在检测过程中不会打断巡线
    qr_detector_node = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='qr_detector',
        name='qr_detector',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'min_area_ratio': 0.005,
            'show_image': True,
            'detect_every_n': 3,
            'detect_scale': 1.0,
            'min_consecutive': 3,
        }],
    )

    # 速度仲裁器: QR 事件优先级高于巡线, 检测到二维码先减速后停下,
    # 再按二维码内容执行 左转/右转/停止/直行
    cmd_arbiter_node = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='cmd_arbiter',
        name='cmd_arbiter',
        parameters=[{
            # 减速 / 停车
            'decel_duration': 1.2,
            'publish_rate': 20.0,
            'detect_timeout': 0.5,
            'clear_hold': 1.0,
            'resume_after_clear': True,
            'stop_dwell': 0.5,
            'same_qr_cooldown': 5.0,
            # 路径动作(左/右转直到重新发现线)
            'enable_path_action': True,
            'turn_angular_speed': 0.4,
            'turn_min_time': 1.0,
            'turn_max_time': 8.0,
            'line_found_eps': 0.005,
            'line_confirm': 3,
            'use_odom_turn': True,
            'odom_topic': '/odom',
        }],
    )

    return LaunchDescription([
        wheeltec_robot,
        wheeltec_camera,
        line_follow_node,
        qr_detector_node,
        cmd_arbiter_node,
    ])
