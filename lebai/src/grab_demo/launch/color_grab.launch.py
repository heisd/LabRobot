import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.launch_description_sources import AnyLaunchDescriptionSource


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
def generate_launch_description():
   
    robot_description_semantic_config = load_file(
        "lebai_lm3_moveit_config", "config/lebai_lm3.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('astra_camera'),
    #             'launch',
    #             'gemini_arm.launch.py'
    #         ])
    #     ])
    # )
    # 1. 使用 get_package_share_directory (立即求值)
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir,'launch')

    # 2. 使用 AnyLaunchDescriptionSource (支持 XML 和 Python)
    # 检查XML文件是否存在，如果不存在则使用Python文件
    gemini_arm_xml = os.path.join(astra_launch_dir,'gemini_arm.launch.xml')
    gemini_arm_py = os.path.join(astra_launch_dir,'gemini_arm.launch.py')
    
    if os.path.exists(gemini_arm_xml):
        camera_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gemini_arm_xml)
        )
    elif os.path.exists(gemini_arm_py):
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gemini_arm_py)
        )
    else:
        # 如果都不存在，使用默认的相机启动
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'),
                    'launch',
                    'gemini.launch.py'
                ])
            ])
        )
    robot_ip = LaunchConfiguration('robot_ip')
    robot_ip='192.168.0.50'
    lebai_lm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lebai_lm3_moveit_config'),
                'launch',
                'lm3.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip
        }.items()
    )

    # 给抓取服务提供颜色识别节点hsv
    color_dectet=Node(
            package="grab_demo",
            executable="hsv_range",
            name="color_node",
        )
    # 提供相机内参节点
    camera_info=Node(
            package="grab_demo",
            executable="camera_info_node",
            name="camera_info",
        )
    # 抓取仲裁: 手动优先, 可随时打断自动抓取
    arm_arbiter=Node(
            package="grab_demo",
            executable="arm_arbiter_node.py",
            name="arm_arbiter",
        )
    # 提供抓取服务节点: 闭环(PBVS)版, 与 yolo_ros_grab / kcf_grab 同一思路
    # (看-动-再看-修正后再抓)。想用回开环把 closed_loop_grab_node 换成 grab_service_node。
    grab_service=Node(
            package="grab_demo",
            executable="closed_loop_grab_node",
            name="grab_service_n",
            parameters=[robot_description_semantic, {
                "base_frame": "base_link",
                "look_target": "look",
                "max_iters": 4,
                "pos_tolerance": 0.008,
                "approach_height": 0.10,
                "grasp_z_offset": 0.02,
                "settle_sec": 0.6,
            }]
        )
    # 延迟15秒后启动抓取服务节点，确保其他节点先初始化完成
    delay_task = TimerAction(period=15.0, actions=[grab_service])


    return LaunchDescription([
            camera_launch,#启动相机
            camera_info,#相机内参发布
            arm_arbiter,#抓取仲裁(手动优先)
            lebai_lm3,#启动机械臂
            color_dectet,#颜色识别节点
            delay_task,
    ])

    