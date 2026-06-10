# wheeltec_dashboard

A lightweight web-based dashboard for the Wheeltec S300 robot. Visualizes
telemetry from `turn_on_wheeltec_robot`, drives the chassis through
`/cmd_vel`, and tunes ROS 2 parameters on the running robot node.

## 组成

- `rosbridge_websocket` — 浏览器 ↔ ROS 2 桥接 (默认 `ws://<host>:9090`)
- 一个 Python 节点 `web_server` — 用 `http.server` 静态托管前端 (默认 `http://<host>:8080`)
- 前端：原生 HTML/CSS/JS + [`roslibjs`](https://github.com/RobotWebTools/roslibjs) + [`Chart.js`](https://www.chartjs.org/) + [`ros3djs`](https://github.com/RobotWebTools/ros3djs) / [`three.js`](https://threejs.org/) (走 CDN)

## 依赖

```bash
sudo apt install ros-humble-rosbridge-suite
# web_video_server 由本 repo 内的 web_video_server-ros2 包提供，随 colcon 一起构建
```

构建：

```bash
colcon build --packages-select wheeltec_dashboard
source install/setup.bash
```

## 使用

```bash
ros2 launch wheeltec_dashboard dashboard.launch.py
```

然后在浏览器打开 `http://<机器人IP>:8080/`，页面默认会连接到
`ws://<同一host>:9090` (rosbridge)。

启动参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `http_port` | `8080` | dashboard HTTP 端口 |
| `ws_port` | `9090` | rosbridge_websocket 端口 |
| `video_port` | `8081` | web_video_server (MJPEG) 端口 |
| `enable_video` | `true` | 是否随 dashboard 拉起 web_video_server |
| `address` | `0.0.0.0` | HTTP 监听地址 |

例：`ros2 launch wheeltec_dashboard dashboard.launch.py http_port:=8000`

## 功能

页面顶部为导航栏，把所有功能分为六个板块，点击切换（也支持 `#overview`、
`#components`、`#control`、`#function`、`#arm`、`#contact` 锚点深链）：

- **系统总览**：3D 视图（雷达 `/scan`）、实时遥测、电压 / cmd_vel 折线图
- **各组件状态**：雷达（双雷达融合健康）、超声波、语音组件、相机预览、
  `/rosout` 日志面板
- **控制模块**：速度控制（遥控）、参数调节
- **功能模块**（含子页面）：巡线、KCF 跟踪、YOLO 检测、VLA 语音导航
- **机械臂**（含子页面）：监控与控制、HSV / YOLO / KCF / ArUco / VLM 五种
  grab_demo 抓取方案
- **联系作者**：项目仓库与反馈渠道

各板块明细：

- **雷达**：订阅融合 `/scan` 与单雷达 `/scan1`/`/scan2`（`sensor_msgs/LaserScan`），
  显示每路在线状态、有效点数与融合最近障碍距离。融合由 `double_lidar_fusion`
  完成；点云可视化见"系统总览"3D 视图。
- **语音组件**：来自 `wheeltec_mic` 的麦克风初始化 `/voice_flag`、唤醒
  `/awake_flag`、声源角 `/awake_angle`、识别 `/voice_words`，并可向
  `/tts_text` 发文本播报（经 `tts_make`）。
- **巡线**（二维码版 `line_follow_qr_fixed`）：显示二维码状态
  （`/qr_code/detected`、`/qr_code/data`、`/qr_code/area_ratio`）与 `cmd_arbiter`
  输出的 `/cmd_vel`，并预览两路调试画面（见下方"OpenCV 窗口转发"）。
- **OpenCV 窗口转发**：视觉节点原本用 `cv2.imshow` 弹本地窗口，现可把那帧
  发布成 ROS Image，经 web_video_server 在浏览器查看（机器人无显示器也能用）：
  - `qr_detector` → `/qr_code/debug_image`（检测 HUD + 二维码框）
  - `line_follow_plain` → `/line_follow/debug_image`（巡线掩码）
  - `wheeltec_robot_kcf` → `/KCF_image`（跟踪框，本就发布）

  由各节点参数 `publish_debug`（默认 `true`）控制；想保留本地 cv2 窗口/trackbar
  加 `show_image:=true`；无 trackbar 时巡线颜色用参数 `line_color`
  （0红/1绿/2蓝/3黄/4黑）。
- **KCF 跟踪**（`wheeltec_robot_kcf`）：预览标注流 `/KCF_image`，并实时调节
  `/image_converter` 的距离/转向 PID 参数（`targetDist_`、`linear_K*_`、
  `angular_K*_`）。目标框需在机器人端 OpenCV 窗口鼠标框选。
- **YOLO 检测**：本仓库未内置 YOLO 节点，提供通用查看器——可订阅任意
  `vision_msgs/msg/Detection2DArray` 检测话题（默认 `/yolo/detections`）并列出
  类别/置信度，图像话题可指向检测节点的标注输出。

- 实时遥测：`/PowerVoltage`、`/robot_charging_flag`、`/robot_charging_current`、
  `/robot_red_flag`、`/self_check_data`、`/odom`、`/imu/data_raw`、`/Distance`
- 速度控制：方向按键 + 线/角速度上限滑块 + 键盘 WASD/空格 (停)
- 参数调节：通过 `rcl_interfaces/GetParameters`/`SetParameters` 服务读写
  `/wheeltec_robot` 上的 `odom_x_scale`、`odom_y_scale`、
  `odom_z_scale_positive`、`odom_z_scale_negative`
- 折线图：电压、cmd_vel (vx / wz)
- **3D 视图（嵌入式 RViz 替代）**：基于 ros3djs，支持 Grid、TF、LaserScan
  (`/scan`)、OccupancyGrid (`/map`)、Odometry 轨迹。Fixed frame 默认
  `odom_combined`。URDF 加载在 ROS 2 + rosbridge 下为实验功能，建议留空。
- **相机预览**：launch 同时拉起 `web_video_server`，dashboard 通过
  MJPEG 同时显示两路相机——默认车上 `/camera/color/image_raw` 与机械臂
  `/camera_arm/color/image_raw`，话题/画质/端口可编辑。深度流把 topic
  改成 `/camera/depth/image_raw` 即可。
- **日志面板**：订阅 `/rosout`，按等级 (DEBUG/INFO/WARN/ERROR/FATAL)
  与节点名子串过滤；可配置环形缓冲行数 (50–5000)、暂停/继续、清空、
  自动滚动；按等级着色。
- **VLA 语音导航**：把自然语言指令发布到 `/vla/instruction`（输入框 +
  常用目的地快捷按钮），让 `vla_navigation` 包的 `vla_navigator` 节点用
  本地多模态大模型决策导航；实时显示麦克风识别 `/voice_words`、语音播报
  `/tts_text`，以及决策/导航时间线 `/vla/status`；还可向 `/tts_text` 发文本
  让小车开口说话。**需先启动 `vla_navigator`（如 `vla_bringup.launch.py`）**，
  目标点可在本面板 3D 视图或 RViz 的 `/goal_pose` 查看。
- **机械臂**（Lebai LM3，`lebai_driver` + `grab_demo`，全部经 rosbridge，
  含子页面）：
  - **监控与控制** 子页：
    - 状态监控：`/robot_status`（急停/上电/可运动/运动中/错误/模式）、
      `/gripper_status`（夹爪位置/力度）、`/joint_states`（关节角度表，
      度+弧度+速度）、`/grab_target/distance`（视觉目标深度）、
      `/arm_arbiter/state`（抓取控制权）；驱动离线 3 秒自动回 "—"。
    - 系统控制：上电/断电/使能/去使能/暂停/恢复/中止/进出示教/关机/急停，
      即 `/system_service/*`（std_srvs/Empty）；危险操作有二次确认，急停
      立即下发。
    - 夹爪控制：位置/力度滑块 + 张开/闭合快捷键，调
      `/io_service/set_gripper_position|set_gripper_force`（SetGripper）。
    - 关节运动：6 关节角(rad) + acc/vel 调 `/motion_service/move_joint`
      （MoveJoint，二次确认），可一键填入当前关节角。
    - 抓取与仲裁（各方案共用）：调 `/obj_grab_service`（GrabObject）抓取
      指定 TF 目标；`/arm_arbiter/manual_takeover|manual_release` 手动
      接管/释放，可打断自动抓取；事件时间线记录所有命令与结果。
    - 机械臂相机：MJPEG 预览 `/camera_arm/color/image_raw`（与组件页共用
      web_video_server 设置）。
  - **抓取方案子页**（对应 `grab_demo` 各 launch，共用 `target_frame` TF +
    `/grab_target/distance` + `/obj_grab_service` 流程，每页含调试画面、
    目标距离与"抓取目标"快捷键）：
    - **HSV 颜色抓取**（`color_grab.launch.py`）：调试图
      `/color_node/detection_image`；**HSV 阈值滑条**（H/S/V min/max +
      min_area，拖动即生效，可"读取当前值"同步），对照调试画面调色。
    - **YOLO 抓取**（`yolo_ros_grab.launch.py`）：调试图
      `/yolo_ros_node/detection_image`（带框+距离）；**识别到的物体渲染成
      按钮**（订阅 `/yolo/detections`，同类合并显示数量与最高置信度），
      **点击即选为抓取目标**（实时写桥接节点 `target_label`），"清除筛选"
      恢复任意类别；`target_label` / `target_class` / `conf_threshold`
      也可手动调参（`/yolo_ros_node`）。
    - **KCF 跟踪抓取**（`kcf_grab.launch.py`）：跟踪画面
      `/kcf_node/tracking_image`，**支持直接在画面上拖拽框选目标**
      （显示坐标换算为图像像素后发 `/kcf_node/select_bbox`，节点立即用
      该框重新播种）；【重新播种 (HSV)】调 `/kcf_node/reinit`；HSV 播种
      阈值滑条（`/kcf_node`）。
    - **ArUco 抓取**（`aruco_grab.launch.py`）：显示机械臂相机原图
      （aruco_node 无调试图）。
    - **VLM 语言抓取**（`vlm_grab.launch.py`）：框选画面
      `/vlm_node/vlm_image`；自然语言指令 `/vlm/instruction` + 理解结果
      `/vlm/result` + 确认/取消 `/vlm/confirm`。
  - **需在机械臂上先启动**：`lebai_driver` 的 `robot_state` / `io_service` /
    `system_service` / `motion`（或任一 `grab_demo` 抓取 launch，已含全套）。

## 网络说明

前端用到的 `roslibjs` 与 `Chart.js` 通过 CDN 加载，第一次使用需联网。
如果是离线环境，把对应 JS 下载到 `web/` 并改 `index.html` 中的引用即可。
