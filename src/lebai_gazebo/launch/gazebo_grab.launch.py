import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
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

    # world->base_link 的 TF 不再单独发布: lm3_gazebo.xacro 已含 world 链接 +
    # world_to_base 固定关节(z=0.035, 与 Gazebo 锚定位置一致),
    # robot_state_publisher 会自动发布该静态 TF, 再发恒等 TF 会冲突。

    # ---- 4) 抓取服务 ----
    grab_service = Node(
        package="grab_demo", executable="grab_service_node", name="grab_service_n",
        parameters=[robot_description_semantic, sim_time],
    )

    # ---- 3) 视觉节点: 按 world 选对应方案的节点, 让仿真像真机一样产出"处理后的图像" ----
    # 各节点把调试/结果图发到 Dashboard 已对接好的话题:
    #   HSV  -> /color_node/detection_image
    #   KCF  -> /kcf_node/tracking_image
    #   YOLO -> /yolo_ros_node/detection_image (需另装 yolo_ros/yolo_bringup)
    # 仿真相机自带 camera_info, 故内参话题指向仿真相机, 不用 camera_info_node。
    def select_vision_nodes(context, *_a, **_kw):
        # 注意: 内层 gazebo.launch.py 会把 world 解析成完整 .world 路径并回写到
        # "world" 配置, 这里取 basename 去扩展名, 兼容传入"短名"或"完整路径"两种情况。
        raw = LaunchConfiguration("world").perform(context).strip()
        world_name = os.path.splitext(os.path.basename(raw))[0]
        sel = "hsv_range(color_node)"
        cam = {
            "rgb_topic": "/camera_arm/color/image_raw",
            "depth_topic": "/camera_arm/depth/image_raw",
            "camera_info_topic": "/camera_arm/color/camera_info",
            "camera_frame": "camera_arm_depth_optical_frame",
            "target_frame": "target_frame",
        }
        if world_name == "grab_kcf":
            # KCF 跟踪: 默认 HSV(红)自动播种, 可在 Dashboard 画面拖拽框选重新播种。
            vision = Node(
                package="grab_demo", executable="kcf_track_node", name="kcf_node",
                parameters=[{
                    **cam,
                    "z_offset": 0.07,
                    "init_bbox": [0, 0, 0, 0],
                    "hue_min": 0, "hue_max": 10,
                    "sat_min": 100, "sat_max": 255,
                    "val_min": 100, "val_max": 255,
                    "min_area": 400,
                    "reinit_on_loss": True,
                    "show_image": False,
                    "publish_debug_image": True,
                }, sim_time],
            )
            sel = "kcf_track_node(kcf_node) -> /kcf_node/tracking_image"
        elif world_name == "grab_yolo":
            # YOLO: grab_demo 侧的 yolo_ros 桥接节点(订阅 /yolo/detections 画框 + 算距离)。
            # 需另行启动 yolo_ros(ros2 launch yolo_bringup yolo_wheeltec.launch.py),
            # 否则本节点正常运行但收不到检测、调试图为空(仿真场景图 /sim_scene/arm 仍照常显示)。
            vision = Node(
                package="grab_demo", executable="yolo_ros_detect_node.py", name="yolo_ros_node",
                parameters=[{
                    **cam,
                    "detections_topic": "/yolo/detections",
                    "z_offset": 0.07,
                    "select_mode": "confidence",
                    "publish_debug_image": True,
                }, sim_time],
            )
            sel = "yolo_ros_detect_node(yolo_ros_node) -> /yolo_ros_node/detection_image (需 yolo_ros)"
        else:
            # grab_world / grab_hsv_color / grab_aruco / grab_vlm:
            # 统一用 HSV(最轻量, 一定有处理后图像), 默认红色阈值正好识别桌上红块/可乐罐。
            vision = Node(
                package="grab_demo", executable="hsv_range", name="color_node",
                parameters=[{**cam}, sim_time],
            )
        # 等 Gazebo / 控制器起来后再起 move_group + 视觉 + 抓取服务
        return [
            LogInfo(msg=f"[gazebo_grab] world='{world_name}' -> 视觉节点: {sel}"),
            TimerAction(period=12.0, actions=[move_group, vision, grab_service]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="grab_world",
            description="抓取世界名: grab_world / grab_hsv_color / grab_yolo / "
                        "grab_kcf / grab_aruco / grab_vlm"),
        gazebo,
        OpaqueFunction(function=select_vision_nodes),
    ])
