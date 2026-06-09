#!/usr/bin/env python3
# coding=utf-8
"""KCF 跟踪(经 cmd_arbiter 仲裁底盘).

与 wheeltec_robot_kcf.launch.py 的区别: KCF 跟踪速度不直接发 /cmd_vel, 而是经
remap 发到 kcf/cmd_vel, 由 cmd_arbiter 仲裁后下发 /cmd_vel. 仲裁优先级:
    键盘(最高)  >  {巡线 / KCF / YOLO 平级}
这样 KCF 跟踪不直接抢 /cmd_vel, 键盘可随时打断(终端打印日志).

跟踪距离: 参数 targetDist_(米), 启动可调, 也可在仪表盘 KCF 页用滑块实时调.

可选键盘接管(另开终端):
  ros2 run wheeltec_robot_keyboard wheeltec_keyboard --ros-args -r cmd_vel:=cmd_vel_keyboard

示例:
  ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf_arbiter.launch.py
  ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf_arbiter.launch.py target_dist:=0.6
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

KCF_TOPIC = 'kcf/cmd_vel'


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    target_dist = LaunchConfiguration('target_dist')

    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')))
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')))

    kcf = Node(
        package='wheeltec_robot_kcf',
        executable='run_tracker_node',
        parameters=[{'targetDist_': ParameterValue(target_dist, value_type=float)}],
        remappings=[('/cmd_vel', KCF_TOPIC)],   # KCF 速度交给仲裁器
        output='screen',
    )
    arbiter = Node(
        package='simple_follower_ros2',
        executable='cmd_arbiter',
        name='cmd_arbiter',
        parameters=[{
            'kcf_topic': KCF_TOPIC,
            'enable_path_action': False,        # 纯跟踪, 无 QR 路径动作
        }],
        output='screen',
    )

    banner = LogInfo(msg=[
        '[KCF] 经 cmd_arbiter: KCF -> ', KCF_TOPIC,
        ' -> /cmd_vel; 优先级 键盘 > 巡线/KCF/YOLO(平级)'])

    return LaunchDescription([
        DeclareLaunchArgument('target_dist', default_value='0.8'),
        banner, wheeltec_camera, wheeltec_robot, kcf, arbiter,
    ])
