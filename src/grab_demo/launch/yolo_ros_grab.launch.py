import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description():
    # =========================================================================
    # 基于官方 yolo_ros (https://github.com/mgonzs13/yolo_ros) 的闭环抓取启动文件。
    #
    #   相机 + 机械臂   : 与 color_grab / yolo_grab 一致
    #   识别(YOLO)      : 直接复用 yolo_ros 的 yolo.launch.py(ultralytics 推理)
    #   桥接            : yolo_ros_detect_node.py 把 /yolo/detections -> target_frame TF
    #   抓取(闭环)      : closed_loop_grab_node 反复看-动-修正后再抓取
    # =========================================================================

    # ---- 可调启动参数 ----
    model_arg = DeclareLaunchArgument(
        "model", default_value="yolov8n.pt",
        description="ultralytics 模型, 如 yolov8n.pt / yolov8m.pt / 自训练 .pt")
    device_arg = DeclareLaunchArgument(
        "device", default_value="cuda:0",
        description="推理设备: Jetson/GPU 用 cuda:0; 纯 CPU 的 cv docker 用 cpu")
    threshold_arg = DeclareLaunchArgument(
        "threshold", default_value="0.5", description="YOLO 置信度阈值")
    target_class_arg = DeclareLaunchArgument(
        "target_class", default_value="-1",
        description="只抓某个 COCO 类 id, -1=不限 (瓶子=39, 杯子=41)")
    target_label_arg = DeclareLaunchArgument(
        "target_label", default_value="",
        description="只抓某个类名(不区分大小写), 非空时优先于 target_class")
    select_mode_arg = DeclareLaunchArgument(
        "select_mode", default_value="confidence",
        description="多目标选择: confidence / nearest / center / largest")

    model = LaunchConfiguration("model")
    device = LaunchConfiguration("device")
    threshold = LaunchConfiguration("threshold")
    target_class = LaunchConfiguration("target_class")
    target_label = LaunchConfiguration("target_label")
    select_mode = LaunchConfiguration("select_mode")

    # 机械臂语义描述(供抓取节点)
    robot_description_semantic_config = load_file(
        "lebai_lm3_moveit_config", "config/lebai_lm3.srdf")
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config}

    # ---- 启动相机 (与 yolo_grab 一致) ----
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
                PathJoinSubstitution([FindPackageShare('astra_camera'),
                                      'launch', 'gemini.launch.py'])
            ]))

    # ---- 启动机械臂 ----
    robot_ip = '192.168.0.50'
    lebai_lm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('lebai_lm3_moveit_config'),
                                  'launch', 'lm3.launch.py'])
        ]),
        launch_arguments={'robot_ip': robot_ip}.items())

    # ---- 官方 yolo_ros 识别 (直接 include 它的 yolo.launch.py) ----
    # 只用 2D 检测(use_3d=False): 3D 反投影 + target_frame 由我们的桥接节点完成,
    # 这样深度/补偿/距离话题都与既有 HSV/KCF/VLM 流程完全一致。
    yolo_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('yolo_bringup'),
                                  'launch', 'yolo.launch.py'])
        ]),
        launch_arguments={
            'model': model,
            'device': device,
            'threshold': threshold,
            'namespace': 'yolo',
            'use_tracking': 'False',
            'use_3d': 'False',
            'use_debug': 'False',          # 调试图由我们的桥接节点画(带距离), 这里关掉省算力
            'input_image_topic': '/camera_arm/color/image_raw',
            'image_reliability': '1',      # 1=RELIABLE, 与 gemini 相机一致
        }.items())

    # ---- 桥接节点: yolo_ros 检测 -> 统一 target_frame TF ----
    yolo_ros_bridge = Node(
        package="grab_demo",
        executable="yolo_ros_detect_node.py",
        name="yolo_ros_node",
        parameters=[{
            "detections_topic": "/yolo/detections",
            "rgb_topic": "/camera_arm/color/image_raw",
            "depth_topic": "/camera_arm/depth/image_raw",
            "camera_info_topic": "/gemini_info",
            "camera_frame": "camera_arm_depth_optical_frame",
            "target_frame": "target_frame",
            "z_offset": 0.07,
            # target_class 是 int 参数, LaunchConfiguration 是字符串, 必须显式声明类型
            "target_class": ParameterValue(target_class, value_type=int),
            "target_label": target_label,
            "select_mode": select_mode,
            "min_dist": 0.1,
            "max_dist": 2.0,
            "publish_debug_image": True,
        }])

    # 相机内参发布
    camera_info = Node(package="grab_demo", executable="camera_info_node", name="camera_info")

    # ---- 闭环抓取服务 (替代开环的 grab_service_node) ----
    closed_loop_grab = Node(
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
        }])
    delay_grab = TimerAction(period=15.0, actions=[closed_loop_grab])

    return LaunchDescription([
        model_arg, device_arg, threshold_arg,
        target_class_arg, target_label_arg, select_mode_arg,
        camera_launch,      # 相机
        camera_info,        # 相机内参
        lebai_lm3,          # 机械臂
        yolo_ros_launch,    # 官方 yolo_ros 识别
        yolo_ros_bridge,    # 检测结果 -> target_frame
        delay_grab,         # 闭环抓取服务
    ])
