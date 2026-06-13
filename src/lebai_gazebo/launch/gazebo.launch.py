import os
import re

import xacro
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('lebai_gazebo')
    xacro_file = os.path.join(pkg, 'urdf', 'lm3_gazebo.xacro')

    # 用 world 参数选择不同功能的抓取世界(文件名即功能):
    #   grab_world(默认/通用) / grab_hsv_color / grab_yolo / grab_kcf / grab_aruco / grab_vlm
    world = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([pkg, 'worlds', [world, '.world']])

    # 用 xacro 展开机器人描述, 并剥掉 XML 注释:
    # gazebo_ros2_control 会把整串 URDF 以 --param robot_description:=<urdf> 传给
    # controller_manager, rcl 的参数解析器遇到注释里的"冒号+空格"会解析失败,
    # 导致 controller_manager 起不来(spawner 一直等不到 /controller_manager)。
    urdf_xml = xacro.process_file(xacro_file).toxml()
    urdf_xml = re.sub(r'<!--.*?-->', '', urdf_xml, flags=re.S)
    robot_description = {
        'robot_description': ParameterValue(urdf_xml, value_type=str)
    }
    # 启动 Gazebo Classic(含 GUI) + 我们的世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # 发布 TF / robot_description(供 spawn 用)。
    # 节点名/话题名特意与默认值区分开: 底盘(wheeltec)侧也跑着名为
    # robot_state_publisher 的节点并 latch /robot_description, 同名时
    # gazebo_ros2_control 查参数、spawn_entity 读话题都可能拿到底盘的 URDF
    # (报 "no ros2_control tag"/生成错模型)。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
        remappings=[('robot_description', 'arm/robot_description')],
    )

    # 把机械臂模型生成到 Gazebo。
    # -timeout 60: WSL 下 gzserver 启动时 OpenAL 探测音频设备偶发卡 30s+,
    # 默认 30s 超时会让 spawn 放弃 -> 机器人不出现, 故放宽。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'arm/robot_description', '-entity', 'lebai_lm3',
                   '-timeout', '60'],
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
        # WSL 下 OpenAL 探测 PulseAudio 设备偶发卡死 30s+, 拖慢 world 加载;
        # 仿真不需要声音, 强制 OpenAL 用 null 后端跳过设备探测。
        SetEnvironmentVariable('ALSOFT_DRIVERS', 'null'),
        DeclareLaunchArgument(
            'world', default_value='grab_world',
            description='抓取世界名(worlds/<name>.world): grab_world / grab_hsv_color / '
                        'grab_yolo / grab_kcf / grab_aruco / grab_vlm'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # 模型生成完再依次激活控制器
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm])),
    ])
