import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(path, "r") as f:
            return f.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    sim_time = {"use_sim_time": True}

    # ---- 1) Gazebo 场景 + 机械臂 + 相机 + 控制器 ----
    # world 参数选择抓取世界(透传给 gazebo.launch.py):
    #   grab_world(默认) / grab_hsv_color / grab_yolo / grab_kcf / grab_aruco / grab_vlm
    world = LaunchConfiguration("world")
    gz_pkg = get_package_share_directory("lebai_gazebo")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    # ---- 2) MoveIt move_group (仿真配置, 复用 lebai_lm3_moveit_config 的参数) ----
    robot_description = {
        "robot_description": xacro.process_file(
            os.path.join(gz_pkg, "urdf", "lm3_gazebo.xacro")
        ).toxml()
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "lebai_lm3_moveit_config", "config/lebai_lm3.srdf")
    }
    kinematics_yaml = load_yaml("lebai_lm3_moveit_config", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/ResolveConstraintFrames "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_yaml = load_yaml("lebai_lm3_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager":
            load_yaml("lebai_lm3_moveit_config", "config/lm3_controllers.yaml"),
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 3.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            sim_time,
        ],
    )

    # grab_service_node 用 base_link 作参考系; 仿真里给一个 world->base_link 恒等 TF
    static_tf_base = Node(
        package="tf2_ros", executable="static_transform_publisher", output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[sim_time],
    )

    # ---- 3) 视觉节点(用 HSV, 默认红色阈值正好识别桌上的可乐罐) ----
    # 仿真相机自带 camera_info, 所以把内参话题指到仿真相机, 不用 camera_info_node
    hsv = Node(
        package="grab_demo", executable="hsv_range", name="color_node",
        parameters=[{
            "rgb_topic": "/camera_arm/color/image_raw",
            "depth_topic": "/camera_arm/depth/image_raw",
            "camera_info_topic": "/camera_arm/color/camera_info",
            "camera_frame": "camera_arm_depth_optical_frame",
            "target_frame": "target_frame",
        }, sim_time],
    )

    # ---- 4) 抓取服务 ----
    grab_service = Node(
        package="grab_demo", executable="grab_service_node", name="grab_service_n",
        parameters=[robot_description_semantic, sim_time],
    )

    # 等 Gazebo / 控制器起来后再起 move_group 与抓取服务
    delayed = TimerAction(period=12.0, actions=[move_group, hsv, grab_service])

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="grab_world",
            description="抓取世界名: grab_world / grab_hsv_color / grab_yolo / "
                        "grab_kcf / grab_aruco / grab_vlm"),
        gazebo,
        static_tf_base,
        delayed,
    ])
