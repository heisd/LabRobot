# Wheeltec S300 移动底盘（src/）

**Wheeltec S300** 移动机器人部分，基于 **ROS2 Humble** 开发，包含底盘驱动、传感器驱动、SLAM 建图、Nav2 导航、视觉/雷达跟随、语音交互、自动回充等完整功能包。所有功能包均位于仓库统一工作空间的
[`src/`](../src) 目录下，与机械臂（[`docs/lebai.md`](lebai.md)）共用同一个 colcon 工作空间。

> 配套常用指令速查见 [`wheeltec/ROS2-V3.5(humble)常用指令.txt`](wheeltec/ROS2-V3.5(humble)常用指令.txt)。
>
> 下位机硬件资料：主控板 **C63A 原理图** 见
> [`firmware/C63A原理图_V2.0_2025-06-16.pdf`](../firmware/C63A原理图_V2.0_2025-06-16.pdf)，
> 配套固件源码（Keil/STM32F407，FreeRTOS）见 [`firmware/KeilSingleChipProject`](../firmware/KeilSingleChipProject)。

---

## 目录

- [环境与依赖](#环境与依赖)
- [编译与运行](#编译与运行)
- [功能包说明](#功能包说明)
- [常用功能命令](#常用功能命令)
- [常用维护命令](#常用维护命令)

---

## 环境与依赖

- **ROS 版本**：ROS2 Humble
- **平台**：Ubuntu 22.04（机器人主机 IP 默认 `192.168.0.100`，用户 `wheeltec`）
- **车型**：通过环境变量 `ROBOT_TYPE` 指定，例如：

  ```bash
  echo 'export ROBOT_TYPE=s300_pro' >> ~/.bashrc
  ```

安装功能包依赖（在工作空间根目录执行）：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

> 外部依赖 `yolo_ros`（YOLO 检测 / 3D 跟随，与 `grab_demo` 共用）以 **git submodule** 形式位于
> `src/yolo_ros`，克隆仓库后执行 `git submodule update --init --recursive` 拉取，再装
> `ultralytics`。详见 [`wheeltec_yolo/README.md`](../src/wheeltec_yolo/README.md)。
>
> 串口通信库 `serial_ros2`（`turn_on_wheeltec_robot` 依赖）同样以 **git submodule** 形式位于
> `src/serial_ros2`，随 `git submodule update --init --recursive` 一并拉取。

---

## 编译与运行

在工作空间根目录（`src` 的上一级）执行：

```bash
# 编译全部功能包
colcon build --packages-skip-build-finished --continue-on-error

# 单独编译某个功能包（以底盘为例）
colcon build --packages-select turn_on_wheeltec_robot

# 设置环境变量
source install/setup.bash
```

> 注意：修改 launch 文件内容后需要重新编译才能生效。

---

## 功能包说明

### 底盘与基础驱动

| 功能包 | 说明 |
| --- | --- |
| `turn_on_wheeltec_robot` | 底盘控制核心包，提供机器人启动、相机/雷达/传感器一键启动 launch |
| `serial_ros2` | 串口通信底层库 |
| `robot_interfaces` | 自定义消息/服务接口 |
| `wheeltec_robot_keyboard` | 键盘遥控节点（`wheeltec_keyboard`） |
| `wheeltec_joy` | USB 手柄遥控 |
| `wheeltec_rviz2` | RViz2 可视化配置 |
| `rm_description` / `wheeltec_dashboard` | 机器人模型描述 / Web 仪表盘（遥测、cmd_vel、参数调优、巡线/KCF/YOLO 视觉流转发与跟踪距离滑块、路径录制/跟随可视化、四种 SLAM 建图页 + RRT 自主探索边界选点，基于 rosbridge + web_video_server） |

### 传感器驱动

| 功能包 | 说明 |
| --- | --- |
| `wheeltec_lidar_ros2` | 激光雷达驱动（含 `LDlidar`、`LSlidar`） |
| `double_lidar_fusion` | 双激光雷达数据融合 |
| `wheeltec_ultrasonic` | 超声波转 Range / PointCloud2，供 Nav2 使用 |
| `wheeltec_imu` | IMU 驱动（`yesense_ros2`） |
| `ros2_astra_camera-master` | Astra 深度相机驱动（`astra_camera`、`astra_camera_msgs`） |
| `usb_cam-ros2` | V4L USB 相机驱动 |
| `web_video_server-ros2` | 浏览器实时视频流服务 |

### SLAM 建图与导航

| 功能包 | 说明 |
| --- | --- |
| `wheeltec_robot_slam` | 2D/3D 建图集合：`slam_gmapping`、`openslam_gmapping`、`wheeltec_cartographer`、`wheeltec_slam_toolbox`、`orb_slam_2_ros-ros2` |
| `wheeltec_robot_nav2` | Nav2 导航（含多点导航、地图保存） |
| `nav2_waypoint_cycle` | Nav2 多航点循环巡航 |
| `wheeltec_robot_rtab` | RTAB-Map 三维建图与导航 |
| `wheeltec_robot_rrt2` / `wheeltec_rrt_msg` | RRT 自主探索建图及其消息接口 |
| `wheeltec_path_follow` | 路径记录与路径跟踪 |
| `auto_recharge_ros2` | 自动回充功能 |

### 视觉 / 跟随 / 交互

| 功能包 | 说明 |
| --- | --- |
| `simple_follower_ros2` | 简单跟随：雷达跟随、视觉巡线（含二维码路径选择）、视觉跟踪；`cmd_arbiter` 速度仲裁（键盘 > 巡线/KCF/YOLO 平级） |
| `wheeltec_robot_kcf` | KCF 视觉目标跟随（需在 ROS 主机上运行）；跟踪距离 `targetDist_` 可实时调，可经 `cmd_arbiter` 仲裁 |
| `wheeltec_yolo` | YOLO 检测 / 3D 跟随：集成 [yolo_ros](https://github.com/mgonzs13/yolo_ros)（git submodule，见 `src/yolo_ros`），保持设定距离，结果接入 Web 仪表盘 |
| `aruco_ros-humble-devel` | ArUco 二维码识别（`aruco`、`aruco_msgs`、`aruco_ros`） |
| `wheeltec_bodyreader` | 人体骨架识别、姿态控制与人体跟随（`bodyreader`、`bodyreader_msg`） |

### 语音 / AI

| 功能包 | 说明 |
| --- | --- |
| `wheeltec_mic` | 麦克风阵列驱动与语音控制（`wheeltec_mic_ros2`、`wheeltec_mic_msg`） |
| `wheeltec_aiui` | 讯飞 AIUI 语音交互 |
| `tts_make_ros2` | 文本转语音（TTS） |
| `ollama_ros_chat` | 基于 Ollama 的本地大模型对话（`ollama_ros_chat`、`ollama_ros_msgs`） |

### 工具 / 其它

| 功能包 | 说明 |
| --- | --- |
| `qt_ros_test` | ROS2 Qt 图形界面示例 |

---

## 常用功能命令

> 启动任意功能前，请先 `source install/setup.bash`。

### 1. 底盘与传感器

```bash
# 打开机器人底盘控制
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
# 打开底盘相机
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
# 打开雷达
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
# 同时打开雷达、相机和底盘
ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py
```

### 2. 遥控

```bash
# 键盘控制
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
# USB 手柄控制
ros2 launch wheeltec_joy wheeltec_joy.launch.py
```

### 3. 跟随功能

```bash
# 雷达跟随
ros2 launch simple_follower_ros2 laser_follower.launch.py
# 视觉巡线
ros2 launch simple_follower_ros2 line_follower.launch.py
# 视觉巡线 + 二维码路径选择（固定转角）
ros2 launch simple_follower_ros2 line_follow_qr_fixed.launch.py
# 视觉跟踪
ros2 launch simple_follower_ros2 visual_follower.launch.py
# KCF 跟随（需在 ROS 主机上运行）
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf.launch.py

# —— YOLO 检测 / 3D 跟随 —— 先拉取 yolo_ros 子模块并装 ultralytics（见 wheeltec_yolo/README.md）
git submodule update --init --recursive   # 仅首次：拉取 src/yolo_ros (yolo_msgs / yolo_ros / yolo_bringup)
# 仅检测（结果给仪表盘 YOLO 页：/yolo/detections、/yolo/debug_image）
ros2 launch wheeltec_yolo yolo.launch.py
# 3D 跟随，保持 ~0.2m（需相机 depth_registration:=true 让深度与彩色对齐）
ros2 launch wheeltec_yolo yolo_follow.launch.py desired_distance:=0.3 target_class:=person
```

#### 经 cmd_arbiter 仲裁底盘

多个控制源共存时不直接抢 `/cmd_vel`，优先级 **键盘（最高） > { 巡线 / KCF / YOLO 三者平级 }**，
键盘打断时终端打印日志：

```bash
# YOLO 跟随经仲裁器（跟随 -> yolo/cmd_vel -> cmd_arbiter -> /cmd_vel）
ros2 launch wheeltec_yolo yolo_follow_arbiter.launch.py
# KCF 跟踪经仲裁器（KCF -> kcf/cmd_vel -> cmd_arbiter -> /cmd_vel）
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf_arbiter.launch.py
# 让键盘走仲裁器（最高优先，打断时打印 “键盘接管, 打断 X”）
ros2 run wheeltec_robot_keyboard wheeltec_keyboard --ros-args -r cmd_vel:=cmd_vel_keyboard
```

> 跟踪 / 跟随距离可在 Web 仪表盘对应页用滑块实时调（YOLO `desired_distance`、
> KCF `targetDist_`），无需重启节点。

### 4. 2D 建图与导航

```bash
# gmapping 建图
ros2 launch slam_gmapping slam_gmapping.launch.py
# cartographer 建图
ros2 launch wheeltec_cartographer cartographer.launch.py
# 保存地图
ros2 launch wheeltec_nav2 save_map.launch.py
# 2D 导航（含多点导航，在 ROS 主机上运行）
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
```

### 5. RTAB-Map 建图与导航

```bash
# 建图
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py
ros2 launch wheeltec_nav2 save_map.launch.py
# 导航
ros2 launch wheeltec_robot_rtab rtabmap_localization.launch.py
ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py
```

### 6. RRT 自主探索建图

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
ros2 launch wheeltec_robot_rrt wheeltec_rrt_slam.launch.py
# 顺/逆时针发布四个点，最后一个点发布在已知地图中
```

### 7. 路径跟踪

```bash
# 记录路径
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
ros2 launch wheeltec_path_follow save_path.launch.py
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
# 开启路径跟踪
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
ros2 launch wheeltec_path_follow follow_path.launch.py
```

### 8. Web 浏览器查看摄像头

> **注意**：仪表盘 launch（`wheeltec_dashboard dashboard.launch.py` /
> `labrobot_bringup.launch.py` / `vla_bringup.launch.py`）已经自带一个
> `web_video_server`（端口 8081）。仪表盘在跑时**不要**再手动
> `ros2 run web_video_server web_video_server`，否则会出现两个同名
> `/web_video_server` 节点（`ros2 node list` 会告警 share an exact name）。

仅在不使用仪表盘、单独看流时才手动启动：

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
ros2 run web_video_server web_video_server
# 浏览器访问 http://192.168.0.100:8080/
```

### 9. 语音 / 交互

```bash
# 文本转语音（文本内容在 tts_make.launch.py 中配置）
ros2 launch tts tts_make.launch.py
# 语音控制：麦克风阵列初始化
ros2 launch wheeltec_mic_ros2 mic_init.launch.py
# 语音控制：小车功能初始化
ros2 launch wheeltec_mic_ros2 base.launch.py
```

### 10. 骨架识别

```bash
# 姿态控制（含多人定向控制、融合 RGB 记忆人物）
ros2 launch bodyreader bodyinteraction.launch.py
# 人体骨架跟随
ros2 launch bodyreader bodyfollow.launch.py
# 姿态控制 + 跟随（双手胸前交叉切换模式）
ros2 launch bodyreader final.launch.py
```

### 11. 自动回充

```bash
# 1) 修改车型与电池容量：编辑 auto_recharge_ros2/robot_info.yaml
# 2) 建图并保存
ros2 launch slam_gmapping slam_gmapping.launch.py
ros2 launch wheeltec_nav2 save_map.launch.py
# 3) 开启导航
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
# 4) 开启自动回充
ros2 run auto_recharge_ros2 auto_recharge
# 在 rviz 上通过话题 charger_position_update 标定充电桩位置
```

### 12. 其它

```bash
# ROS2 Qt 功能
ros2 launch qt_ros_test qt_ros_test.launch.py
# ORB-SLAM
ros2 launch orb_slam2_ros orb_slam2_Astra_rgbd_launch.py
```

---

## 常用维护命令

```bash
# 查看图像话题
rqt_image_view
# 查看节点与话题关系
rqt_graph
# 生成 TF 树 pdf（生成在当前终端路径下）
ros2 run tf2_tools view_frames
# 手动保存地图
ros2 run nav2_map_server map_saver_cli -f ~/map
# 给文件夹下所有文件可执行权限
sudo chmod -R 777 文件夹
# ssh 登录机器人
ssh -Y wheeltec@192.168.0.100
# vnc 调整分辨率
xrandr --fb 1024x768
```

---

> 更多详细操作请参考各功能包内文档及官方功能手册。
