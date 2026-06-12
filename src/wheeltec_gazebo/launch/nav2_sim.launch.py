"""一键启动"Gazebo 导航仿真 + Nav2" —— /goal_pose 即真正驱动 Nav2。

做了三件事:
  1. 起 Gazebo + 导航世界(默认 wheeltec_nav) + 仿真底盘(gazebo.launch.py);
  2. 把 wheeltec_nav2 的真机参数自动适配仿真: 真机里程计坐标系是 EKF 的
     odom_combined, 仿真 diff_drive 发布的是 odom —— 启动时整体替换后写出
     仿真参数文件再传给 Nav2(原文件不动);
  3. 起 Nav2(wheeltec_nav2/bringup_launch.py, use_sim_time:=True)。
     默认 slam:=True: 用 slam_toolbox 边建图边导航, **无需先存地图、也无需设
     初始位姿**(SLAM 自己发布 map→odom), 面板 Nav2 页直接发 /goal_pose 就能动;
     已存好仿真世界的地图时用 slam:=False map:=/路径/xx.yaml 切换 AMCL 定位
     (此时需先在面板发"初始位姿")。

例:
  ros2 launch wheeltec_gazebo nav2_sim.launch.py
  ros2 launch wheeltec_gazebo nav2_sim.launch.py world:=wheeltec_vla_nav
  ros2 launch wheeltec_gazebo nav2_sim.launch.py slam:=False map:=$HOME/sim_map.yaml

依赖(运行的机器上): ros-humble-nav2-bringup ros-humble-slam-toolbox
配合面板: ros2 launch wheeltec_dashboard sim_bringup.launch.py (面板在 :8000),
或在面板"功能模块→导航 Nav2"页一键启停本 launch(sim_launcher 的 nav 项)。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _nav2(context, *_args, **_kw):
    """启动时把真机 Nav2 参数适配仿真(odom_combined→odom), 再包含 Nav2 bringup。"""
    nav2_dir = get_package_share_directory('wheeltec_nav2')
    robot_type = os.getenv('ROBOT_TYPE', 's300_pro').lower()
    src_params = os.path.join(nav2_dir, 'param', f'nav_param_{robot_type}.yaml')

    with open(src_params, 'r', encoding='utf-8') as f:
        text = f.read()
    # 真机: EKF 发布 odom_combined→base_footprint; 仿真: diff_drive 发布 odom→base_footprint。
    # 参数里的坐标系/话题名(amcl odom_frame_id / bt_navigator odom_topic /
    # local_costmap global_frame 等)统一替换。
    text = text.replace('odom_combined', 'odom')
    sim_params = os.path.join('/tmp', f'nav_param_{robot_type}_sim.yaml')
    with open(sim_params, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'[nav2_sim] 仿真 Nav2 参数已生成: {sim_params} (odom_combined→odom)')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'True',
            'params_file': sim_params,
        }.items(),
    )]


def generate_launch_description():
    pkg = get_package_share_directory('wheeltec_gazebo')
    nav2_dir = get_package_share_directory('wheeltec_nav2')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='wheeltec_nav',
            description='导航世界: wheeltec_nav / wheeltec_slam_nav / wheeltec_vla_nav 等'),
        DeclareLaunchArgument('x', default_value='-3.0', description='底盘出生点 X'),
        DeclareLaunchArgument('y', default_value='-3.0', description='底盘出生点 Y'),
        DeclareLaunchArgument(
            'slam', default_value='True',
            description='True=slam_toolbox 边建图边导航(免地图/免初始位姿); '
                        'False=用 map 参数的已存地图 + AMCL 定位(需发初始位姿)'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(nav2_dir, 'map', 'WHEELTEC.yaml'),
            description='slam:=False 时加载的地图 yaml(请换成给仿真世界建好的图; '
                        '默认的 WHEELTEC.yaml 是真实实验室的图, 与仿真世界不符)'),
        gazebo,
        OpaqueFunction(function=_nav2),
    ])
