import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('lebai_gazebo')
    xacro_file = os.path.join(pkg, 'urdf', 'lm3_gazebo.xacro')
    world = os.path.join(pkg, 'worlds', 'grab_world.world')
    
    # 用 xacro 实时展开机器人描述
    robot_description = {
    'robot_description': ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    # 启动 Gazebo Classic(含 GUI) + 我们的世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    # 发布 TF / robot_description(供 spawn 用)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 把机械臂模型生成到 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'lebai_lm3'],
        output='screen',
    )

    # ros2_control 控制器(由 gazebo_ros2_control 插件起的 controller_manager 加载)
    load_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    load_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['lebai_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # 模型生成完再依次激活控制器
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm])),
    ])
