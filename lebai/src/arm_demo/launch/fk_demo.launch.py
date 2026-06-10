# 导入所需的模块
import os  # 用于与操作系统交互
import yaml  # 用于加载和解析YAML文件
from launch import LaunchDescription  # 用于定义launch文件
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution  # 用于从启动参数、路径中获取值以及文本替换
from launch.conditions import IfCondition, UnlessCondition  
from launch_ros.actions import Node  # 用于启动ROS节点
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 用于从另一个Python语言编写的launch文件中引用生成launch文件
from launch_ros.substitutions import FindPackageShare  # 用于查找ROS软件包的共享目录
from ament_index_python.packages import get_package_share_directory  # 用于获取ROS软件包的共享目录

# 定义一个函数，用于加载文件并返回其内容
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)  # 获取软件包的共享目录路径
    absolute_file_path = os.path.join(package_path, file_path)  # 将共享目录路径和文件路径拼接成绝对路径

    try:  # 尝试打开文件并读取其中的内容
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # 如果出现错误，返回空值
        return None

# 定义一个函数，用于加载YAML文件并返回其内容
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)  # 获取软件包的共享目录路径
    absolute_file_path = os.path.join(package_path, file_path)  # 将共享目录路径和文件路径拼接成绝对路径

    try:  # 尝试打开文件并解析其中的内容为Python对象
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # 如果出现错误，返回空值
        return None

# 定义生成launch文件的函数
def generate_launch_description():
   
    robot_description_semantic_config = load_file(  # 加载SRDF文件的内容
        "lebai_lm3_moveit_config", "config/lebai_lm3.srdf"
    )
    robot_description_semantic = {  # 将SRDF内容作为参数传递给节点
        "robot_description_semantic": robot_description_semantic_config
    }

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

    fk_node=Node(
            package="arm_demo",  # 节点所在的软件包名
            executable="fk_demo",  # 节点的可执行文件名
            name="arm_fk",  # 节点名称
            parameters=[  # 节点参数
                robot_description_semantic  # 机器人模型参数
            ]
        )
    
    return LaunchDescription([  # 返回启动的节点
        lebai_lm3,
        fk_node,

    ])