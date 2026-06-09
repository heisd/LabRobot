import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
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
    # 与 color_grab.launch.py 相同的相机 + 机械臂启动逻辑,
    # 把检测节点换成 KCF 跟踪节点(kcf_track_node)。
    robot_description_semantic_config = load_file(
        "lebai_lm3_moveit_config", "config/lebai_lm3.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # ---- 启动相机 ----
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
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'), 'launch', 'gemini.launch.py'
                ])
            ])
        )

    # ---- 启动机械臂(含 MoveIt + 驱动) ----
    lebai_lm3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lebai_lm3_moveit_config'), 'launch', 'lm3.launch.py'
            ])
        ]),
        launch_arguments={'robot_ip': '192.168.0.50'}.items()
    )

    # ---- KCF 跟踪节点(替代 hsv_range / yolo, 输出同样的 target_frame) ----
    kcf_track = Node(
        package="grab_demo",
        executable="kcf_track_node",
        name="kcf_node",
        parameters=[{
            "rgb_topic": "/camera_arm/color/image_raw",
            "depth_topic": "/camera_arm/depth/image_raw",
            "camera_info_topic": "/gemini_info",
            "camera_frame": "camera_arm_depth_optical_frame",
            "target_frame": "target_frame",
            "z_offset": 0.07,
            # 初始框: [x,y,w,h], 全 0 表示用 HSV 自动播种
            "init_bbox": [0, 0, 0, 0],
            # HSV 自动播种阈值(默认偏红色物体, 按需修改)
            "hue_min": 0, "hue_max": 10,
            "sat_min": 100, "sat_max": 255,
            "val_min": 100, "val_max": 255,
            "min_area": 400,
            "reinit_on_loss": True,
            "show_image": False,
            "publish_debug_image": True,
        }]
    )

    camera_info = Node(
        package="grab_demo", executable="camera_info_node", name="camera_info",
    )
    # 闭环(PBVS)抓取: 与 yolo_ros_grab 同一思路, 看-动-再看-修正后再抓。
    # KCF 持续跟踪并刷新 target_frame, 闭环节点据此反复修正机械臂位姿。
    # 想用回开环, 把 closed_loop_grab_node 换成 grab_service_node 即可(服务名一致)。
    grab_service = Node(
        package="grab_demo", executable="closed_loop_grab_node", name="grab_service_n",
        parameters=[robot_description_semantic, {
            "base_frame": "base_link",
            "look_target": "look",
            "max_iters": 4,
            "pos_tolerance": 0.008,
            "approach_height": 0.10,
            "grasp_z_offset": 0.02,
            "settle_sec": 0.6,
        }]
    )
    delay_task = TimerAction(period=15.0, actions=[grab_service])

    return LaunchDescription([
        camera_launch,   # 启动相机
        camera_info,     # 相机内参发布
        lebai_lm3,       # 启动机械臂
        kcf_track,       # KCF 跟踪节点
        delay_task,      # 闭环抓取服务
    ])
