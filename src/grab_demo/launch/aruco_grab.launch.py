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
    # 使用 get_package_share_directory 和 AnyLaunchDescriptionSource 加载 XML 文件
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir, 'launch')
    
    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_launch_dir, 'gemini_arm.launch.xml'))
    )

    robot_ip = LaunchConfiguration('robot_ip')
    robot_ip='192.168.0.50'  #机械臂ip
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


    aruco_dectet=Node(
            package="grab_demo",
            executable="aruco_dectet",
            name="aruco_node",
            parameters=[
                {'markerLength': 0.05},#标签边长
                # arm机械臂相机话题 
                # 修改下面的aruco_dectet.cpp文件中的rgb_image_topic话题名
                # 就需要更改这里的参数
                {'rgb_image_topic': '/camera_arm/color/image_raw'},#订阅相机rgb话题
                {'rgb_image_info': '/gemini_info'},#订阅相机内参话题
                {'dectet_result_image_pub': '/aruco_dect_image'},#识别结果图像话题名
                {'dictionary_id': 6},# aruco字典
                {'reference_frame':'camera_color_optical_frame'},# 参考坐标
                {'object_frame':'aruco_link'}#目标坐标
            ],
        )

    camera_info=Node(
            package="grab_demo",
            executable="camera_info_node",
            name="camera_info",
        )
    
    grab_service=Node(
            package="grab_demo",
            executable="grab_service_node",
            name="grab_service",
            parameters=[robot_description_semantic]
        )
    delay_task=TimerAction(period=15.0,actions=[grab_service])


    return LaunchDescription([
            camera_launch,#启动相机
            camera_info,#相机内参发布节点
            lebai_lm3,#启动机械臂
            aruco_dectet,#aruco识别节点
            delay_task,
    ])

    
