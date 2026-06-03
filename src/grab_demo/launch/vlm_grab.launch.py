import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description():
    # 相机 + 机械臂 与其它视觉抓取一致, 检测节点换成 VLM(视觉语言模型)抓取节点。
    robot_description_semantic_config = load_file(
        "lebai_lm3_moveit_config", "config/lebai_lm3.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # ---- 相机 ----
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir, 'launch')
    gemini_arm_xml = os.path.join(astra_launch_dir, 'gemini_arm.launch.xml')
    gemini_arm_py = os.path.join(astra_launch_dir, 'gemini_arm.launch.py')
    if os.path.exists(gemini_arm_xml):
        camera_launch = IncludeLaunchDescription(AnyLaunchDescriptionSource(gemini_arm_xml))
    elif os.path.exists(gemini_arm_py):
        camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gemini_arm_py))
    else:
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'), 'launch', 'gemini.launch.py'
                ])
            ])
        )

    # ---- 机械臂(MoveIt + 驱动) ----
    lebai_lm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lebai_lm3_moveit_config'), 'launch', 'lm3.launch.py'
            ])
        ]),
        launch_arguments={'robot_ip': '192.168.0.50'}.items()
    )

    # ---- VLM 抓取节点 ----
    vlm_node = Node(
        package="grab_demo",
        executable="vlm_grab_node.py",
        name="vlm_node",
        output="screen",
        parameters=[{
            "rgb_topic": "/camera_arm/color/image_raw",
            "depth_topic": "/camera_arm/depth/image_raw",
            "camera_info_topic": "/gemini_info",
            "camera_frame": "camera_arm_depth_optical_frame",
            "target_frame": "target_frame",
            "z_offset": 0.07,
            # VLM 接口: "openai"(兼容本地 Ollama/vLLM 或云端) 或 "anthropic"
            "provider": "openai",
            "api_base": "https://api.openai.com/v1",
            "model": "gpt-4o-mini",
            # api key 从环境变量读取(OPENAI_API_KEY / ANTHROPIC_API_KEY)
            "auto_grab": True,           # 理解到目标后是否调用抓取服务
            "require_confirm": True,     # 安全: 抓取前需 Dashboard 二次确认
            "min_dist": 0.1,            # 安全: 允许的最近/最远距离(m)
            "max_dist": 1.5,
            "max_instruction_len": 200,  # 安全: 指令长度上限
            "force_json": True,          # 安全: openai 强制 JSON 输出(本地服务不支持则设 False)
            "grab_service": "/obj_grab_service",
            "instruction_topic": "/vlm/instruction",
            "result_topic": "/vlm/result",
            "confirm_topic": "/vlm/confirm",
            "publish_debug_image": True,
        }]
    )

    camera_info = Node(package="grab_demo", executable="camera_info_node", name="camera_info")
    grab_service = Node(
        package="grab_demo", executable="grab_service_node", name="grab_service_n",
        parameters=[robot_description_semantic]
    )
    delay_task = TimerAction(period=15.0, actions=[grab_service])

    return LaunchDescription([
        camera_launch,
        camera_info,
        lebai_lm3,
        vlm_node,
        delay_task,
    ])
