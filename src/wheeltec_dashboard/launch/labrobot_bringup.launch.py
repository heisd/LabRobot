"""一键启动 LabRobot 整套系统：底盘 + 雷达 + 双相机 + 语音 + Web 仪表盘（可选机械臂）。

    ros2 launch wheeltec_dashboard labrobot_bringup.launch.py

设计目标
========
1. 一条命令拉起整个系统（各组件都有 start_* 开关可单独关掉）。
2. 故障隔离：某个组件出问题时报错打印到终端，但不影响其余组件启动——
   * 生成阶段：include 经 _opt_include() 预检（包是否存在、launch 能否解析），
     失败只打 [labrobot_bringup] 警告并跳过该组件，其余照常；
   * 运行阶段：ROS 2 launch 对节点崩溃本来就不连坐（这里不设 on_exit=Shutdown），
     报错持续显示在终端；sensor_watchdog（随仪表盘启动）还会把相机/雷达/IMU/
     下位机/机械臂的掉线以 ERROR 写入 /rosout，面板日志卡可见。
3. 不重复启动节点：
   * web_video_server 只由 dashboard.launch.py 启动一个（:8081），
     不要再手动 `ros2 run web_video_server web_video_server`，否则会出现两个
     同名 /web_video_server 节点；
   * 机械臂驱动默认随 bringup 启动（start_arm 默认 true，含 MoveIt），这样面板
     一开机就能收到 /robot_status 判机械臂在线。但 grab_demo 的各抓取 launch
     （color_grab / yolo_grab / …）自带同一套 lebai 驱动 + 机械臂相机，叠加会重复：
     要单独跑抓取 launch 时，先用 start_arm:=false（并按需 start_arm_camera:=false）
     关掉本侧驱动/相机，否则会出现重名驱动节点、相机“设备占用”等冲突。

分阶段启动（与 vla_bringup 一致的错峰思路）：
  t=0s 底盘 turn_on_wheeltec_robot（串口驱动 + EKF→odom_combined TF + URDF）
  t=2s 雷达（双雷达 + 融合）、车上相机 /camera/*、机械臂相机 /camera_arm/*、
       超声波转换（底盘 Distance -> /ultrasonic/A..F + /ultrasonic/points）
  t=4s Web 仪表盘（rosbridge :9090 + http :8000 + web_video_server :8081 + watchdog）
  t=6s 语音（麦克风阵列 + 离线识别 + TTS）、AI 对话(ollama_ros_chat)、可选 lebai 机械臂

导航/VLA 仍用 vla_navigation/vla_bringup.launch.py（其 start_base 等开关可与
本文件错开，避免重复启动底盘）。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opt_node(label, **node_kwargs):
    """构建一个"可失败"的 Node：所属包缺失时打印警告并跳过（share 目录存在
    与否作为包已安装的探针，缺包时 Node 在运行期才报错会拖死整个 launch）。"""
    try:
        get_package_share_directory(node_kwargs['package'])
        return [Node(**node_kwargs)]
    except Exception as exc:  # noqa: BLE001  组件级故障隔离，刻意宽抓
        msg = f'[labrobot_bringup] 组件 "{label}" 不可用，已跳过: {exc}'
        print(msg, file=sys.stderr)
        return [LogInfo(msg=msg)]


def _opt_include(label, package, relpath, launch_arguments=None, condition=None):
    """构建一个"可失败"的 include：包缺失/launch 解析失败 → 警告并跳过。

    普通 IncludeLaunchDescription 在被包含文件出错（如其内部
    get_package_share_directory 找不到包）时会把整个 launch 拖死；这里把
    解析提前到生成阶段做一次预检，失败只影响该组件自己。
    """
    try:
        share = get_package_share_directory(package)
        path = os.path.join(share, relpath)
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        source = AnyLaunchDescriptionSource(path)
        probe = getattr(source, 'try_get_launch_description_without_context', None)
        if probe is not None and probe() is None:
            raise RuntimeError(
                f'解析 {path} 失败（其依赖的包可能未编译或未 source）')
        kwargs = {}
        if launch_arguments is not None:
            kwargs['launch_arguments'] = launch_arguments.items()
        if condition is not None:
            kwargs['condition'] = condition
        return [IncludeLaunchDescription(source, **kwargs)]
    except Exception as exc:  # noqa: BLE001  组件级故障隔离，刻意宽抓
        msg = f'[labrobot_bringup] 组件 "{label}" 不可用，已跳过: {exc}'
        print(msg, file=sys.stderr)
        return [LogInfo(msg=msg)]


def generate_launch_description():
    start_base = LaunchConfiguration('start_base')
    start_lidar = LaunchConfiguration('start_lidar')
    start_camera = LaunchConfiguration('start_camera')
    start_arm_camera = LaunchConfiguration('start_arm_camera')
    start_ultrasonic = LaunchConfiguration('start_ultrasonic')
    start_voice = LaunchConfiguration('start_voice')
    start_dashboard = LaunchConfiguration('start_dashboard')
    start_arm = LaunchConfiguration('start_arm')

    declare_args = [
        DeclareLaunchArgument('start_base', default_value='true',
                              description='底盘 turn_on_wheeltec_robot（串口+EKF+URDF TF）'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='双雷达 + double_lidar_fusion 融合(/scan)'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='车上相机 Gemini(/camera/color/image_raw)'),
        DeclareLaunchArgument('start_arm_camera', default_value='true',
                              description='机械臂相机 Gemini(/camera_arm/color/image_raw)；'
                                          '抓取 launch 自带相机，叠加会报设备占用'),
        DeclareLaunchArgument('start_ultrasonic', default_value='true',
                              description='超声波距离转换 supersonic_converter（把底盘 Distance '
                                          '拆成 /ultrasonic/A..F + /ultrasonic/points，面板超声波卡据此'
                                          '判活）；机型经 ROBOT_TYPE 环境变量，默认 s300_mini'),
        DeclareLaunchArgument('start_voice', default_value='true',
                              description='麦克风阵列 + 离线识别 + TTS'),
        DeclareLaunchArgument('start_dashboard', default_value='true',
                              description='Web 仪表盘(rosbridge+http+web_video_server+watchdog)'),
        DeclareLaunchArgument('start_arm', default_value='true',
                              description='lebai LM3 机械臂驱动(含 MoveIt)，发布 /robot_status 等，'
                                          '面板据此判机械臂在线。grab_demo 抓取 launch 自带同一套驱动，'
                                          '要单独跑抓取 launch 前先 start_arm:=false 避免重复'),
        DeclareLaunchArgument('start_llm', default_value='true',
                              description='AI 对话后端 ollama_ros_chat(/chat_service 服务 + '
                                          'topic_server 流式)。需本机 ollama 在跑，'
                                          '没跑只报错不影响其它组件'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.50',
                              description='lebai 机械臂 IP(start_arm:=true 时用)'),
        DeclareLaunchArgument('http_port', default_value='8000',
                              description='仪表盘 HTTP 端口'),
        DeclareLaunchArgument('ws_port', default_value='9090',
                              description='rosbridge websocket 端口'),
        DeclareLaunchArgument('video_port', default_value='8081',
                              description='web_video_server MJPEG 端口'),
    ]

    # ---- t=0s 底盘（EKF 发布 odom_combined→base_footprint，3D 视图的 fixed frame 用它）
    base = _opt_include(
        '底盘', 'turn_on_wheeltec_robot', 'launch/turn_on_wheeltec_robot.launch.py',
        condition=IfCondition(start_base))

    # ---- t=2s 传感器
    lidar = _opt_include(
        '雷达', 'turn_on_wheeltec_robot', 'launch/wheeltec_lidar.launch.py',
        condition=IfCondition(start_lidar))
    car_camera = _opt_include(
        '车上相机', 'turn_on_wheeltec_robot', 'launch/wheeltec_camera.launch.py',
        condition=IfCondition(start_camera))
    arm_camera = _opt_include(
        '机械臂相机', 'astra_camera', 'launch/gemini_arm.launch.xml',
        condition=IfCondition(start_arm_camera))
    # 超声波转换：依赖底盘发布的 Distance(robot_interfaces/Supersonic)，故与传感器同批起
    ultrasonic = _opt_include(
        '超声波转换', 'wheeltec_ultrasonic', 'launch/supersonic+converter.launch.py',
        condition=IfCondition(start_ultrasonic))

    # ---- t=4s 仪表盘（rosbridge + 静态页 + 唯一的 web_video_server + watchdog）
    dashboard = _opt_include(
        '仪表盘', 'wheeltec_dashboard', 'launch/dashboard.launch.py',
        launch_arguments={
            'http_port': LaunchConfiguration('http_port'),
            'ws_port': LaunchConfiguration('ws_port'),
            'video_port': LaunchConfiguration('video_port'),
        },
        condition=IfCondition(start_dashboard))

    # ---- t=6s 语音 + 可选机械臂
    voice = (_opt_include(
        '麦克风语音', 'wheeltec_mic_ros2', 'launch/mic_init.launch.py',
        condition=IfCondition(start_voice))
        + _opt_include(
        'TTS', 'tts', 'launch/tts_make.launch.py',
        condition=IfCondition(start_voice)))
    arm = _opt_include(
        '机械臂', 'lebai_lm3_moveit_config', 'launch/lm3.launch.py',
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')},
        condition=IfCondition(start_arm))

    # AI 对话后端：服务模式(/chat_service) + 话题流式模式(topic_server)，
    # 面板"AI 对话"页签两种 Ollama 后端都能用；DeepSeek API 模式走浏览器直连，
    # 不需要车端节点。
    start_llm = LaunchConfiguration('start_llm')
    llm = (_opt_include(
        'AI对话(服务)', 'ollama_ros_chat', 'launch/ollama_ros_chat.launch.py',
        condition=IfCondition(start_llm))
        + _opt_node(
        'AI对话(流式)', package='ollama_ros_chat', executable='topic_server',
        name='ollama_topic_server', output='screen',
        condition=IfCondition(start_llm)))

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    for action in base:
        ld.add_action(action)
    ld.add_action(TimerAction(period=2.0,
                              actions=lidar + car_camera + arm_camera + ultrasonic))
    ld.add_action(TimerAction(period=4.0, actions=dashboard))
    ld.add_action(TimerAction(period=6.0, actions=voice + arm + llm))
    return ld
