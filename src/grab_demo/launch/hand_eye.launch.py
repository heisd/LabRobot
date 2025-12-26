import os
import yaml
from launch import LaunchDescription
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



    hand_eye=Node(
            package="grab_demo",
            executable="hand_eye",
            name="hand_eye_node",
            parameters=[
            robot_description_semantic,
            {'hand_eye_cout':20},
            ]
        )


    camera_info=Node(
            package="grab_demo",
            executable="camera_info_node",
            name="camera_info",
        )

    # 这个是charuco标定板识别节点
    charuco_dectet=Node(
            package="grab_demo",
            executable="charuco_dectet_node",
            name="dectet",
             parameters=[
                {'squaresX': 5},#charuco纵向个数
                {'squaresY': 7},#charuco横向个数
                # 实际上测量的值 @li
                # 按照实际测量的值为准
                {'squareLength': 0.04},#单个棋盘格边长
                {'markerLength': 0.025},#单个aruco码边长
                # 修改一下这个订阅的话题名 @li
                {'rgb_image_topic': '/camera_arm/color/image_raw'},#订阅的rgb话题名
                {'rgb_image_info': '/gemini_info'},#相机内参话题名
                {'rgb_result_pub': '/charuco_dect'},#识别结果发布话题名
                {'dictionary_id': 6}#aruco字典
            ],
             output="screen"
         
        )


    return LaunchDescription([
            camera_launch,#启动相机
            camera_info,#相机内参发布节点
            charuco_dectet,#charuco标定板识别节点
            lebai_lm3,#启动机械臂
            hand_eye,#手眼标定节点，订阅相机话题名以及相机内参话题名，订阅charuco位姿信息，控制机械臂采样计算手眼位姿
    ])

    
