import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的路径
    mic_package_dir = get_package_share_directory('wheeltec_mic_ros2')
    
    # 声明参数
    use_bluetooth_arg = DeclareLaunchArgument(
        'use_bluetooth',
        default_value='false',
        description='Whether to use bluetooth audio device'
    )
    
    # 获取参数值
    use_bluetooth = LaunchConfiguration('use_bluetooth')

    # arecord -l 可以查看麦克风的设备名
    # 默认 Wheeltec 麦克风阵列: "hw:CARD=XFMDPV0018,DEV=0"
    # 蓝牙设备通常可以使用 "default" 或 "plughw:1,0" 等
    # 注意：本机器人默认没有内置蓝牙板卡，若要使用无线音频，需自行安装蓝牙模块/USB适配器
    
    wheeltec_mic = Node(
        package="wheeltec_mic_ros2",
        executable="wheeltec_mic",
        output='screen',
        # 串口名 根据实际的麦克风来决定
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}]
    )

    # 蓝牙模式下的语音控制节点
    voice_control_bluetooth = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        output='screen',
        name='voice_control',
        parameters=[{
            "appid": "6159904a",
            "source_path": mic_package_dir,
            "audio_device": "default", # 蓝牙模式下建议使用 default
        }],
        condition=IfCondition(use_bluetooth)
    )

    # 有线模式下的语音控制节点
    voice_control_wired = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        output='screen',
        name='voice_control',
        parameters=[{
            "appid": "6159904a",
            "source_path": mic_package_dir,
            "audio_device": "hw:CARD=XFMDPV0018,DEV=0",
        }],
        condition=UnlessCondition(use_bluetooth)
    )

    ld = LaunchDescription()

    ld.add_action(use_bluetooth_arg)
    ld.add_action(wheeltec_mic)
    ld.add_action(voice_control_wired)
    ld.add_action(voice_control_bluetooth)
    
    return ld
