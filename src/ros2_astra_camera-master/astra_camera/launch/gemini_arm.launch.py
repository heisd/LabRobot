import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    # 获取 astra_camera 包的路径
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir, 'launch')
    
    # 加载 XML launch 文件
    gemini_arm_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(astra_launch_dir, 'gemini_arm.launch.xml')
        )
    )
    
    return LaunchDescription([
        gemini_arm_launch,
    ])
