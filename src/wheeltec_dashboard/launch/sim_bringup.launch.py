"""一键拉起"Gazebo 仿真 + Dashboard 面板"。

把 dashboard(rosbridge + web_server + web_video_server + sim_launcher)一次起好;
可选用 ``sim`` 参数顺带自动起一个仿真世界(也可不起、改在网页"仿真启停"卡里选世界启动)。

例:
  # 只起面板(含 sim_launcher), 然后在网页里选世界点"启动仿真"
  ros2 launch wheeltec_dashboard sim_bringup.launch.py

  # 面板 + 自动起底盘"巡线"世界
  ros2 launch wheeltec_dashboard sim_bringup.launch.py sim:=chassis chassis_world:=wheeltec_line_follow

  # 面板 + 自动起机械臂"YOLO 抓取"世界
  ros2 launch wheeltec_dashboard sim_bringup.launch.py sim:=arm arm_world:=grab_yolo

  # 面板 + 自动起"Gazebo 导航仿真 + Nav2"(slam 边建图边导航, /goal_pose 直接可用)
  ros2 launch wheeltec_dashboard sim_bringup.launch.py sim:=nav

打开 http://<本机IP>:8000/ , 顶部"仿真 (Gazebo)"分组即可看转发画面与启停。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _maybe_sim(context, *_args, **_kw):
    """按 sim 参数选择性地包含 Gazebo 仿真(只在需要时解析对应包, 避免没装时报错)。"""
    sim = LaunchConfiguration('sim').perform(context).strip().lower()
    if sim in ('', 'none', 'no', 'off'):
        return []
    if sim == 'chassis':
        pkg = get_package_share_directory('wheeltec_gazebo')
        src = os.path.join(pkg, 'launch', 'gazebo.launch.py')
        world = LaunchConfiguration('chassis_world').perform(context)
    elif sim == 'arm':
        pkg = get_package_share_directory('lebai_gazebo')
        src = os.path.join(pkg, 'launch', 'gazebo_grab.launch.py')
        world = LaunchConfiguration('arm_world').perform(context)
    elif sim == 'nav':
        # Gazebo 导航世界 + Nav2(slam_toolbox 边建图边导航), /goal_pose 即驱动 Nav2
        pkg = get_package_share_directory('wheeltec_gazebo')
        src = os.path.join(pkg, 'launch', 'nav2_sim.launch.py')
        world = LaunchConfiguration('nav_world').perform(context)
    else:
        raise RuntimeError(f"未知 sim 值: {sim!r} (可选 none / chassis / arm / nav)")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(src),
        launch_arguments={'world': world}.items(),
    )]


def generate_launch_description():
    dash = get_package_share_directory('wheeltec_dashboard')

    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dash, 'launch', 'dashboard.launch.py')),
        launch_arguments={
            'http_port': LaunchConfiguration('http_port'),
            'ws_port': LaunchConfiguration('ws_port'),
            'video_port': LaunchConfiguration('video_port'),
            'enable_watchdog': LaunchConfiguration('enable_watchdog'),
            'enable_sim_launcher': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='none',
            description='是否随面板自动起仿真: none(只起面板, 网页里选世界) / chassis / arm / '
                        'nav(Gazebo 导航世界 + Nav2)'),
        DeclareLaunchArgument(
            'chassis_world', default_value='wheeltec_slam_nav',
            description='sim:=chassis 时的底盘世界(wheeltec_slam_nav/rrt_explore/line_follow/target_follow/vla_nav/nav)'),
        DeclareLaunchArgument(
            'arm_world', default_value='grab_world',
            description='sim:=arm 时的抓取世界(grab_world/grab_hsv_color/grab_yolo/grab_kcf/grab_aruco/grab_vlm)'),
        DeclareLaunchArgument(
            'nav_world', default_value='wheeltec_nav',
            description='sim:=nav 时的导航世界(wheeltec_nav/wheeltec_slam_nav/wheeltec_vla_nav)'),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('video_port', default_value='8081'),
        DeclareLaunchArgument(
            'enable_watchdog', default_value='false',
            description='仿真默认关掉真实传感器看门狗(仿真没有真实串口设备)'),
        dashboard,
        OpaqueFunction(function=_maybe_sim),
    ])
