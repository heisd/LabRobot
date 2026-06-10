#!/usr/bin/env python3
# coding=utf-8
"""YOLO 3D 跟随(经 cmd_arbiter 仲裁底盘).

与 yolo_follow.launch.py 的区别: 跟随速度不直接发 /cmd_vel, 而是发到 yolo/cmd_vel,
由 cmd_arbiter 仲裁后下发 /cmd_vel. 仲裁优先级:
    键盘(最高)  >  {巡线 / KCF / YOLO 平级}
这样 YOLO 跟随不直接抢 /cmd_vel, 且键盘可随时打断(终端打印日志).

可选键盘接管(另开一个终端):
  ros2 run wheeltec_robot_keyboard wheeltec_keyboard --ros-args -r cmd_vel:=cmd_vel_keyboard

示例:
  ros2 launch wheeltec_yolo yolo_follow_arbiter.launch.py
  ros2 launch wheeltec_yolo yolo_follow_arbiter.launch.py desired_distance:=0.3 target_class:=person
"""

import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

YOLO_TOPIC = 'yolo/cmd_vel'


def generate_launch_description():
    target_class = LaunchConfiguration('target_class')
    desired_distance = LaunchConfiguration('desired_distance')
    device = LaunchConfiguration('device')
    model = LaunchConfiguration('model')

    declared = [
        DeclareLaunchArgument('target_class', default_value=''),
        DeclareLaunchArgument('desired_distance', default_value='0.20'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
    ]

    follow_launch = os.path.join(
        get_package_share_directory('wheeltec_yolo'), 'launch', 'yolo_follow.launch.py')
    include_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(follow_launch),
        launch_arguments={
            'cmd_vel_topic': YOLO_TOPIC,        # 跟随速度交给仲裁器
            'target_class': target_class,
            'desired_distance': desired_distance,
            'device': device,
            'model': model,
        }.items(),
    )

    arbiter = launch_ros.actions.Node(
        package='simple_follower_ros2',
        executable='cmd_arbiter',
        name='cmd_arbiter',
        parameters=[{
            'yolo_topic': YOLO_TOPIC,
            'enable_path_action': False,   # 纯跟随, 无 QR 路径动作
        }],
        output='screen',
    )

    banner = LogInfo(msg=[
        '[wheeltec_yolo] YOLO 跟随经 cmd_arbiter: 跟随 -> ', YOLO_TOPIC,
        ' -> /cmd_vel; 优先级 键盘 > 巡线/KCF/YOLO(平级)'])

    return LaunchDescription([*declared, banner, include_follow, arbiter])
