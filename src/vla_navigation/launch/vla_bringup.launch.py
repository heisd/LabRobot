"""一键启动: 底盘 + 雷达 + 导航 + 摄像头 + 语音 + VLA 大脑.

默认拉起一整套(各部分都可用 start_* 参数单独关掉):
  底盘 start_base    : turn_on_wheeltec_robot (base_serial/ekf/imu/TF)
  雷达 start_lidar   : wheeltec_lidar
  导航 start_nav     : wheeltec_nav2/bringup_launch.py (map=WHEELTEC.yaml)
  摄像头 start_camera : Orbbec Gemini (wheeltec_camera, 发布 /camera/color/image_raw)
  语音 start_voice   : wheeltec_mic + voice_control + call_recognition + tts
  VLA(始终启动)      : vla_navigator

注意: 故意不启动 command_recognition —— 它会把"去I/J/K点"直接发成 goal_pose,
与 VLA 节点抢导航目标; VLA 模式下由 vla_navigator 统一决定去哪。

示例:
  ros2 launch vla_navigation vla_bringup.launch.py
  # 已经单独开了 Nav2 和摄像头时:
  ros2 launch vla_navigation vla_bringup.launch.py start_nav:=false start_camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vla_share = get_package_share_directory('vla_navigation')
    mic_share = get_package_share_directory('wheeltec_mic_ros2')
    tts_share = get_package_share_directory('tts')
    robot_share = get_package_share_directory('turn_on_wheeltec_robot')
    nav_share = get_package_share_directory('wheeltec_nav2')

    robot_launch = os.path.join(robot_share, 'launch')
    nav_launch = os.path.join(nav_share, 'launch')

    vla_params = os.path.join(vla_share, 'config', 'vla_params.yaml')
    vla_waypoints = os.path.join(vla_share, 'config', 'waypoints.yaml')
    mic_param = os.path.join(mic_share, 'config', 'param.yaml')

    default_map = os.path.join(nav_share, 'map', 'WHEELTEC.yaml')
    default_nav_params = os.path.join(nav_share, 'param', 'nav_param_s300_pro.yaml')

    mic_port = LaunchConfiguration('mic_port')
    mic_baud = LaunchConfiguration('mic_baud')
    asr_appid = LaunchConfiguration('asr_appid')
    vlm_model = LaunchConfiguration('vlm_model')
    use_action = LaunchConfiguration('use_action')
    start_base = LaunchConfiguration('start_base')
    start_lidar = LaunchConfiguration('start_lidar')
    start_nav = LaunchConfiguration('start_nav')
    start_camera = LaunchConfiguration('start_camera')
    start_voice = LaunchConfiguration('start_voice')
    nav_map = LaunchConfiguration('map')
    nav_params = LaunchConfiguration('nav_params')

    declare_args = [
        DeclareLaunchArgument('mic_port', default_value='/dev/wheeltec_mic',
                              description='麦克风阵列串口设备'),
        DeclareLaunchArgument('mic_baud', default_value='115200',
                              description='麦克风串口波特率'),
        DeclareLaunchArgument('asr_appid', default_value='6159904a',
                              description='讯飞离线识别 appid'),
        DeclareLaunchArgument('vlm_model', default_value='qwen2.5vl:3b',
                              description='Ollama 多模态模型名(需已 ollama pull)'),
        DeclareLaunchArgument('use_action', default_value='false',
                              description='true=用 navigate_to_pose action 下发; false=发 goal_pose'),
        DeclareLaunchArgument('start_base', default_value='true',
                              description='启动底盘 turn_on_wheeltec_robot'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='启动激光雷达'),
        DeclareLaunchArgument('start_nav', default_value='true',
                              description='启动 Nav2 导航'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='启动 Orbbec Gemini 摄像头'),
        DeclareLaunchArgument('start_voice', default_value='true',
                              description='启动麦克风/离线识别/TTS 语音链'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Nav2 地图文件'),
        DeclareLaunchArgument('nav_params', default_value=default_nav_params,
                              description='Nav2 参数文件'),
    ]

    # ---------------- 底盘 / 雷达 / 导航 / 摄像头 ----------------
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch, 'turn_on_wheeltec_robot.launch.py')),
        condition=IfCondition(start_base),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch, 'wheeltec_lidar.launch.py')),
        condition=IfCondition(start_lidar),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch, 'wheeltec_camera.launch.py')),
        condition=IfCondition(start_camera),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch, 'bringup_launch.py')),
        launch_arguments={
            'map': nav_map,
            'use_sim_time': 'false',
            'params_file': nav_params,
        }.items(),
        condition=IfCondition(start_nav),
    )

    # ---------------- 语音输入链 + TTS ----------------
    voice_cond = IfCondition(start_voice)
    wheeltec_mic = Node(
        package='wheeltec_mic_ros2', executable='wheeltec_mic', output='screen',
        condition=voice_cond,
        parameters=[{
            'usart_port_name': mic_port,
            'serial_baud_rate': ParameterValue(mic_baud, value_type=int),
        }],
    )
    voice_control = Node(
        package='wheeltec_mic_ros2', executable='voice_control', output='screen',
        condition=voice_cond,
        parameters=[{'appid': asr_appid}],
    )
    call_recognition = Node(
        package='wheeltec_mic_ros2', executable='call_recognition', output='screen',
        condition=voice_cond,
        parameters=[mic_param],
    )
    tts = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tts_share, 'launch', 'tts_make.launch.py')),
        condition=voice_cond,
    )

    # ---------------- VLA 大脑 ----------------
    vla_navigator = Node(
        package='vla_navigation', executable='vla_navigator', name='vla_navigator',
        output='screen',
        parameters=[
            vla_params,
            {
                'waypoints_file': vla_waypoints,
                'vlm_model': vlm_model,
                'use_action': ParameterValue(use_action, value_type=bool),
            },
        ],
    )

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(base)
    ld.add_action(lidar)
    ld.add_action(nav)
    ld.add_action(camera)
    ld.add_action(wheeltec_mic)
    ld.add_action(voice_control)
    ld.add_action(call_recognition)
    ld.add_action(tts)
    ld.add_action(vla_navigator)
    return ld
