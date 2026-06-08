"""一键启动: 语音输入链 + TTS + VLA 导航大脑.

包含的节点:
  - wheeltec_mic      麦克风阵列驱动(唤醒/声源角度/串口)
  - voice_control     讯飞离线识别引擎(/get_offline_result_srv)
  - call_recognition  唤醒后触发识别, 把结果发到 voice_words
  - tts               文本转语音(订阅 tts_text, aplay 播放), 复用 tts 包 launch
  - vla_navigator     指令+画面 -> Ollama 多模态 -> Nav2 目标 + 语音反馈

不在本 launch 内(按各自方式单独启动, 因为依赖地图/硬件):
  - Nav2:  ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py
  - 摄像头: ros2 launch usb_cam usb_cam_launch.py   (发布 /image_raw)

注意: 故意不启动 command_recognition —— 它会把"去I/J/K点"直接发成 goal_pose,
与 VLA 节点抢导航目标; VLA 模式下由 vla_navigator 统一决定去哪。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vla_share = get_package_share_directory('vla_navigation')
    mic_share = get_package_share_directory('wheeltec_mic_ros2')
    tts_share = get_package_share_directory('tts')

    vla_params = os.path.join(vla_share, 'config', 'vla_params.yaml')
    vla_waypoints = os.path.join(vla_share, 'config', 'waypoints.yaml')
    mic_param = os.path.join(mic_share, 'config', 'param.yaml')

    mic_port = LaunchConfiguration('mic_port')
    mic_baud = LaunchConfiguration('mic_baud')
    asr_appid = LaunchConfiguration('asr_appid')
    vlm_model = LaunchConfiguration('vlm_model')
    use_action = LaunchConfiguration('use_action')

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
                              description='true=用 navigate_to_pose action 下发目标; false=发 goal_pose'),
    ]

    # ---------------- 语音输入链 ----------------
    wheeltec_mic = Node(
        package='wheeltec_mic_ros2', executable='wheeltec_mic', output='screen',
        parameters=[{
            'usart_port_name': mic_port,
            'serial_baud_rate': ParameterValue(mic_baud, value_type=int),
        }],
    )
    voice_control = Node(
        package='wheeltec_mic_ros2', executable='voice_control', output='screen',
        parameters=[{'appid': asr_appid}],
    )
    call_recognition = Node(
        package='wheeltec_mic_ros2', executable='call_recognition', output='screen',
        parameters=[mic_param],
    )

    # ---------------- 语音输出(TTS) ----------------
    tts = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tts_share, 'launch', 'tts_make.launch.py')),
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
    ld.add_action(wheeltec_mic)
    ld.add_action(voice_control)
    ld.add_action(call_recognition)
    ld.add_action(tts)
    ld.add_action(vla_navigator)
    return ld
