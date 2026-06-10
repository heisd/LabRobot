import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description():
    # 与 color_grab.launch.py 完全相同的相机 + 机械臂启动逻辑,
    # 只是把 HSV 颜色识别节点换成 YOLO TensorRT 检测节点。
    robot_description_semantic_config = load_file(
        "lebai_lm3_moveit_config", "config/lebai_lm3.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # ---- 启动相机 (与 color_grab 一致) ----
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir, 'launch')
    gemini_arm_xml = os.path.join(astra_launch_dir, 'gemini_arm.launch.xml')
    gemini_arm_py = os.path.join(astra_launch_dir, 'gemini_arm.launch.py')

    if os.path.exists(gemini_arm_xml):
        camera_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gemini_arm_xml)
        )
    elif os.path.exists(gemini_arm_py):
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gemini_arm_py)
        )
    else:
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'),
                    'launch',
                    'gemini.launch.py'
                ])
            ])
        )

    # ---- 启动机械臂 ----
    robot_ip = '192.168.0.50'
    lebai_lm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lebai_lm3_moveit_config'),
                'launch',
                'lm3.launch.py'
            ])
        ]),
        launch_arguments={'robot_ip': robot_ip}.items()
    )

    # ---- 模型路径 (按需修改为你自己的路径) ----
    # 推荐: 把 yolov8n.onnx 放在 ~/models/ 下, 节点首次运行会自动构建并缓存 yolov8n.engine
    model_dir = os.path.expanduser('~/models')
    onnx_path = os.path.join(model_dir, 'yolov8n.onnx')
    engine_path = os.path.join(model_dir, 'yolov8n.engine')

    # ---- YOLO 检测节点 (替代 hsv_range, 输出同样的 target_frame) ----
    yolo_detect = Node(
        package="grab_demo",
        executable="yolo_detect_node",
        name="yolo_node",
        parameters=[{
            "engine_path": engine_path,
            "onnx_path": onnx_path,
            # 相机话题与 HSV 完全一致
            "rgb_topic": "/camera_arm/color/image_raw",
            "depth_topic": "/camera_arm/depth/image_raw",
            "camera_info_topic": "/gemini_info",
            # 输出 TF 与 HSV 完全一致, grab_service_node 无需改动
            "camera_frame": "camera_arm_depth_optical_frame",
            "target_frame": "target_frame",
            # 检测参数: target_class=-1 表示任意类别取最高分;
            # 想只抓某一类(例如瓶子)就设成对应 COCO id, 瓶子=39
            "target_class": -1,
            # 多物体选择策略:
            #   "confidence"=置信度最高  "nearest"=离相机最近
            #   "center"=最靠近画面中心  "largest"=检测框最大
            "select_mode": "confidence",
            "conf_threshold": 0.25,
            "nms_threshold": 0.45,
            "z_offset": 0.07,
            # 默认不弹窗(headless 安全); 需要可视化时用 rqt_image_view 订阅
            #   /yolo_node/detection_image
            "show_image": False,
            "publish_debug_image": True,
        }]
    )

    # 提供相机内参节点
    camera_info = Node(
        package="grab_demo",
        executable="camera_info_node",
        name="camera_info",
    )

    # 提供抓取服务节点
    grab_service = Node(
        package="grab_demo",
        executable="grab_service_node",
        name="grab_service_n",
        parameters=[robot_description_semantic]
    )
    delay_task = TimerAction(period=15.0, actions=[grab_service])

    return LaunchDescription([
        camera_launch,    # 启动相机
        camera_info,      # 相机内参发布
        lebai_lm3,        # 启动机械臂
        yolo_detect,      # YOLO 识别节点(替代 HSV)
        delay_task,
    ])
