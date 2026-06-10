import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """巡线 + 二维码路径选择(固定转角版).

    与 line_follow_qr.launch.py 的区别:
      - 巡线节点用 line_follow_plain(纯巡线, 无任何分叉处理);
      - 配合 path:left30 / path:right30 这类二维码做"固定转角".
    cmd_arbiter 同时兼容 path:left/right(转到发现线)与 path:left30/right30(固定转角).
    """
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
    )
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    # 纯巡线节点(无分叉): 速度重映射到中间话题交给仲裁器
    line_follow_node = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='line_follow_plain',
        name='line_follow_plain',
        remappings=[('cmd_vel', 'line_follow/cmd_vel')],
    )

    # QR 检测节点
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

    # 速度仲裁器: QR 优先, 先减速后停下, 再按内容执行固定转角 / 寻线转角 / 停止 / 直行
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
            'stop_dwell': 0.5,
            'same_qr_cooldown': 5.0,
            'enable_path_action': True,
            'turn_angular_speed': 0.4,
            'turn_min_time': 1.0,
            'turn_max_time': 8.0,
            'line_found_eps': 0.005,
            'line_confirm': 3,
            # 固定转角用里程计闭环, 角度更准
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
