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

页面顶部为导航栏，把所有功能分为五个板块，点击切换（也支持 `#overview`、
`#components`、`#control`、`#function`、`#contact` 锚点深链）：

- **系统总览**：3D 视图（雷达 `/scan`）、实时遥测、电压 / cmd_vel 折线图
- **各组件状态**：雷达（双雷达融合健康）、超声波、语音组件、相机预览、
  `/rosout` 日志面板
- **控制模块**：速度控制（遥控）、参数调节
- **功能模块**（含子页面）：巡线、KCF 跟踪、YOLO 检测、VLA 语音导航
- **联系作者**：项目仓库与反馈渠道

各板块明细：

- **雷达**：订阅融合 `/scan` 与单雷达 `/scan1`/`/scan2`（`sensor_msgs/LaserScan`），
  显示每路在线状态、有效点数与融合最近障碍距离。融合由 `double_lidar_fusion`
  完成；点云可视化见"系统总览"3D 视图。
- **语音组件**：来自 `wheeltec_mic` 的麦克风初始化 `/voice_flag`、唤醒
  `/awake_flag`、声源角 `/awake_angle`、识别 `/voice_words`，并可向
  `/tts_text` 发文本播报（经 `tts_make`）。
- **巡线**（`simple_follower_ros2`）：web_video_server 预览
  `/camera/color/image_raw`，显示巡线节点输出的 `/cmd_vel`。颜色在机器人端
  trackbar 选择。
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

## 网络说明

前端用到的 `roslibjs` 与 `Chart.js` 通过 CDN 加载，第一次使用需联网。
如果是离线环境，把对应 JS 下载到 `web/` 并改 `index.html` 中的引用即可。
