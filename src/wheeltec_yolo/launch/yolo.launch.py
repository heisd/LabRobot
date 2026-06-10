#!/usr/bin/env python3
# coding=utf-8
"""Wheeltec 便捷启动: 包装 yolo_ros 的 ``yolo_bringup/yolo.launch.py``.

只做一件事: 把上游 YOLO 启动的若干参数换成 Wheeltec 的默认值, 再原样转发,
这样现场只需 ``ros2 launch wheeltec_yolo yolo.launch.py`` 一条命令即可.

默认 (均可在命令行用 ``key:=value`` 覆盖):
  - input_image_topic : /camera/color/image_raw  (Wheeltec 相机)
  - model             : yolov8n.pt               (机器人板载/实时友好)
  - device            : cuda:0                    (NVIDIA GPU / Jetson)
  - use_tracking      : True   -> /yolo/tracking 带跟踪 id
  - use_debug         : True   -> /yolo/debug_image 标注流 (仪表盘预览)
  - use_3d            : False  (无需深度)

仪表盘「YOLO 检测」页: 图像话题用 /yolo/debug_image, 检测话题用 /yolo/detections
(yolo_msgs/DetectionArray).

前置: 已 ``vcs import src < yolo_ros.repos`` 拉取并 colcon 编译 yolo_ros, 且相机
节点在发布 /camera/color/image_raw.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Wheeltec 默认值; 键名与上游 yolo.launch.py 的 DeclareLaunchArgument 一致,
    # 因此声明后可直接透传, 现场用 key:=value 覆盖任意一项.
    defaults = {
        'input_image_topic': '/camera/color/image_raw',
        # 深度相关 (use_3d:=True 时生效). Astra 深度需与彩色对齐:
        # 启动相机时加 depth_registration:=true, 否则 3D 位置不准.
        'input_depth_topic': '/camera/depth/image_raw',
        'input_depth_info_topic': '/camera/depth/camera_info',
        'target_frame': 'base_link',
        'model': 'yolov8n.pt',
        'device': 'cuda:0',
        'threshold': '0.5',
        'use_tracking': 'True',
        'use_debug': 'True',
        'use_3d': 'False',
        'namespace': 'yolo',
    }
    declared = [DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()]

    upstream_launch = os.path.join(
        get_package_share_directory('yolo_bringup'), 'launch', 'yolo.launch.py')
    include_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(upstream_launch),
        launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items(),
    )

    # 启动日志: 现场一眼看清这次用的输入/模型/设备/是否开 3D
    banner = LogInfo(msg=[
        '[wheeltec_yolo] 启动 YOLO  input=', LaunchConfiguration('input_image_topic'),
        '  model=', LaunchConfiguration('model'),
        '  device=', LaunchConfiguration('device'),
        '  use_3d=', LaunchConfiguration('use_3d'),
        '  ->  /yolo/detections, /yolo/debug_image'])

    return LaunchDescription([*declared, banner, include_yolo])
