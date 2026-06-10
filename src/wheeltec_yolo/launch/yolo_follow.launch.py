#!/usr/bin/env python3
# coding=utf-8
"""YOLO 3D 跟随: yolo_ros(开启深度 3D) + yolo_follow 节点.

让小车跟随被 YOLO 检测到的物体, 并保持 ~desired_distance(默认 0.20m).

前置:
  1) vcs import src < yolo_ros.repos 并 colcon 编译 (yolo_msgs/yolo_ros/yolo_bringup);
  2) 安装 ultralytics (GPU/Jetson 见 README);
  3) 相机已发布彩色 /camera/color/image_raw 与深度 /camera/depth/image_raw
     (深度需与彩色对齐, 即开启 depth registration; 否则 3D 位置不准).

示例:
  ros2 launch wheeltec_yolo yolo_follow.launch.py
  ros2 launch wheeltec_yolo yolo_follow.launch.py target_class:=person desired_distance:=0.3
  ros2 launch wheeltec_yolo yolo_follow.launch.py cmd_vel_topic:=cmd_vel device:=cpu
"""

import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_class = LaunchConfiguration('target_class')
    desired_distance = LaunchConfiguration('desired_distance')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    device = LaunchConfiguration('device')
    model = LaunchConfiguration('model')

    declared = [
        DeclareLaunchArgument('target_class', default_value=''),         # 空 = 任意类别
        DeclareLaunchArgument('desired_distance', default_value='0.20'),  # 米
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
    ]

    # 复用本包的便捷启动, 打开 3D(深度) 与 tracking/debug
    yolo_launch = os.path.join(
        get_package_share_directory('wheeltec_yolo'), 'launch', 'yolo.launch.py')
    include_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo_launch),
        launch_arguments={
            'use_3d': 'True',
            'use_tracking': 'True',
            'use_debug': 'True',
            'device': device,
            'model': model,
        }.items(),
    )

    follower = launch_ros.actions.Node(
        package='wheeltec_yolo',
        executable='yolo_follow',
        name='yolo_follow',
        parameters=[{
            'detections_topic': '/yolo/detections_3d',
            'target_class': ParameterValue(target_class, value_type=str),
            'desired_distance': ParameterValue(desired_distance, value_type=float),
        }],
        remappings=[('cmd_vel', cmd_vel_topic)],
    )

    # 启动日志
    banner = LogInfo(msg=[
        '[wheeltec_yolo] YOLO 3D 跟随  class=', target_class,
        '  keep=', desired_distance, 'm  cmd_vel->', cmd_vel_topic,
        '  device=', device, '  model=', model,
        '  (深度需 depth_registration:=true, 输入 /yolo/detections_3d)'])

    return LaunchDescription([*declared, banner, include_yolo, follower])
