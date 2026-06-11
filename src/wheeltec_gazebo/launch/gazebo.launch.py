"""启动 Wheeltec 底盘的 Gazebo(Classic) 仿真。

用 world 参数选择不同功能的世界(文件名即带功能名, 方便辨认):
  - wheeltec_slam_nav     建图 / 导航 / 避障(默认)
  - wheeltec_rrt_explore  RRT 自主探索 / 迷宫
  - wheeltec_line_follow  巡线
  - wheeltec_target_follow 目标跟随(KCF/YOLO/骨架) / YOLO 检测
  - wheeltec_vla_nav      VLA 语音导航 / 航点导航 / 路径跟随

例:
  ros2 launch wheeltec_gazebo gazebo.launch.py
  ros2 launch wheeltec_gazebo gazebo.launch.py world:=wheeltec_line_follow
  ros2 launch wheeltec_gazebo gazebo.launch.py world:=wheeltec_rrt_explore x:=-3.0 y:=-3.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('wheeltec_gazebo')
    xacro_file = os.path.join(pkg, 'urdf', 'wheeltec_gazebo.xacro')

    world = LaunchConfiguration('world')
    use_gui = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')

    # 由 world 参数拼出 worlds/<name>.world 的完整路径
    world_path = PathJoinSubstitution([pkg, 'worlds', [world, '.world']])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'gui': use_gui,
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'wheeltec_chassis',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.05', '-Y', spawn_yaw,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='wheeltec_slam_nav',
            description='世界名(worlds/<name>.world): wheeltec_slam_nav / '
                        'wheeltec_rrt_explore / wheeltec_line_follow / '
                        'wheeltec_target_follow / wheeltec_vla_nav'),
        DeclareLaunchArgument('gui', default_value='true', description='是否打开 Gazebo GUI'),
        DeclareLaunchArgument('x', default_value='0.0', description='底盘出生点 X'),
        DeclareLaunchArgument('y', default_value='0.0', description='底盘出生点 Y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='底盘出生朝向(rad)'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
