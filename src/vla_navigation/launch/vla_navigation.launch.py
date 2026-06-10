import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vla_navigation')
    params_file = os.path.join(pkg_share, 'config', 'vla_params.yaml')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    vla_node = Node(
        package='vla_navigation',
        executable='vla_navigator',
        name='vla_navigator',
        output='screen',
        parameters=[params_file, {'waypoints_file': waypoints_file}],
    )

    return LaunchDescription([vla_node])
