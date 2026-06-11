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

只启动仪表盘：

```bash
ros2 launch wheeltec_dashboard dashboard.launch.py
```

一键拉起整套系统（底盘 + 双雷达 + 双相机 + 语音 + 仪表盘，组件出错只报
不连坐，详见文件头注释；各组件有 `start_*` 开关）：

```bash
ros2 launch wheeltec_dashboard labrobot_bringup.launch.py
```

然后在浏览器打开 `http://<机器人IP>:8080/`，页面默认会连接到
`ws://<同一host>:9090` (rosbridge)。

> 仪表盘 launch 已自带一个 `web_video_server`（:8081），不要再手动
> `ros2 run web_video_server web_video_server`，否则出现两个同名节点。
>
> 3D 视图的 Fixed frame 默认 `odom_combined`：底盘 TF 树的根由 EKF 发布为
> `odom_combined→base_footprint→base_link→…`；`/odom` 只是里程计**话题**名，
> TF 里并没有名为 `odom` 的 frame。

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

页面顶部为导航栏，按机器人本体的两个部分 —— **Wheeltec 底盘** 与
**Lebai 机械臂** —— 分组（也支持 `#overview`、`#components`、`#control`、
`#function`、`#arm`、`#chat`、`#contact` 锚点深链）。顶栏右侧常驻
**机械臂在线徽标**（`/robot_status` 3 秒内有数据=在线绿、断流=离线红、
未连 rosbridge=灰）：

- **系统总览**：系统架构卡（底盘 / 机械臂两部分的模块总览 + 在线状态点 +
  点击跳转）、3D 视图（雷达 `/scan`）、实时遥测、电压 / cmd_vel 折线图、
  相机原始流卡（车上 `/camera/color/image_raw` + 机械臂
  `/camera_arm/color/image_raw` 两路 MJPEG，端口/画质与"组件状态"页共用）、
  系统日志卡（`/rosout` 镜像，独立等级/节点过滤；底部四卡 flex 自适应，
  任何屏宽都铺满不留空白）
- **Wheeltec 底盘**
  - **组件状态**：**传感器在线状态指示灯墙 + 串口设备表**（sensor_watchdog：
    车载相机/雷达1/雷达2/IMU/下位机 STM32/Lebai 机械臂/机械臂相机 绿红灯，
    udev 别名→占用串口→USB 设备 ID）、下位机 STM32F407（示意图 +
    串口在线检测 + 固件使能位 + 低压禁动 + 回充模式回读 + 事件日志）、
    雷达（双雷达融合健康）、超声波、语音组件、相机预览、`/rosout` 日志面板
  - **底盘控制**：速度控制（遥控 + 安全等级）、自动回充、RGB 灯带、参数调节
  - **功能模块**（含子页面）：巡线、KCF 跟踪、YOLO 检测、骨架识别、
    VLA 语音导航（含航点标定助手）
- **Lebai 机械臂**
  - **监控与抓取**（含子页面）：监控与控制、HSV / YOLO / KCF / ArUco / VLM
    五种 grab_demo 抓取方案
- **AI 对话**：与大模型文字聊天，三种后端可切——Ollama·ROS 服务
  （`/chat_service`，ollama_ros_chat）、Ollama·ROS 话题流式
  （`/chat_message`→`/chat_response` 逐字渲染）、**DeepSeek API 联网直连**
  （OpenAI 兼容 SSE 流式，API Key 仅存本浏览器 localStorage）
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
- **骨架识别 / 体感跟随**（`wheeltec_bodyreader`，Astra Body Tracking）：
  骨架叠加画面 `/body/body_display`（MJPEG）；订阅 `/body_posture` 显示
  锁定状态（无人/检测到/已锁定）、锁定 ID、目标距离与横向偏角、活跃姿态
  （叉腰锁定/举手/平举/抬脚）与跌倒告警，`/bodylist` 显示视野人数；
  按钮发布 `/mode`（1=姿态交互 2=跟随，切跟随需确认）与 `/recoveryid`
  （找回锁定目标）；`/body_follower` 的 bodyfollow_x_p/x_d/z_p/z_d PID
  可在线调。启动 `ros2 launch bodyreader bodyfollow.launch.py`。
- **下发命令解析**：驱动把发给下位机的 11 字节控制帧回发到
  `/robot_serial_tx`（需重编译），面板按通信协议表解析模式选择位
  （0=速度控制 / 1、2=自动回充 / 3=红外对接速度 / 4=灯带 RGB）、目标速度
  与 BCC 校验——STM32 卡"最近下发指令"实时刷新，命令类型变化写入
  下位机事件栏。
- **地图航点管理**（VLA 子页）：自动加载已建好的地图（`/map_server/map`
  GetMap 服务 + `/map` 话题兜底，SLAM 建图中实时刷新），画布上叠加小车
  实时位姿（绿箭头）、已存航点（蓝点）；**按下选点、拖动定朝向**（同 RViz
  2D Goal Pose）或"用当前位姿"，填名字【保存到机器人】→ 后端
  `vla_navigator` 经 `/vla/waypoint_cmd` **立即生效**并整表持久化到
  `~/.ros/vla_waypoints.yaml`（重启优先加载，一次标定永久有效）；航点列表
  （`/vla/waypoints` 广播）支持【导航】（直发 `/goal_pose`）与【删除】；
  备用"生成 YAML 片段"手动流保留。
- **导航仲裁**（`nav_arbiter`，vla_navigation 包）：面板遥控/WASD 的速度
  同步发 `/cmd_vel_manual`，仲裁节点收到即取消 `navigate_to_pose` 全部
  目标——**手动随时打断 Nav2 与 VLA**，手动期间自主目标插不进来，松手
  约 2s 后自动恢复；状态显示在 VLA 卡（`/nav_arbiter/status`），接管/释放
  写入 VLA 时间线。实体键盘节点 remap `cmd_vel:=cmd_vel_manual` 即可参与。

- 实时遥测：`/PowerVoltage`、`/robot_charging_flag`、`/robot_charging_current`、
  `/robot_red_flag`（**回充红外信号**——固件回充帧 rx[3] 是收到充电桩红外的
  对管个数，不是急停）、`/self_check_data`（新固件该字段恒 0）、`/odom`、
  `/imu/data_raw`、`/Distance`、`/robot_enable_flag`（固件使能位 en_flag，
  需重编译驱动）、`/robot_recharge_mode`（固件回充模式回读，需重编译驱动）
- 速度控制：方向按键 + 线/角速度上限滑块 + 键盘 WASD/空格 (停)；
  **安全等级**开关发布 `/chassis_security`（0=速度流中断自动停车 /
  1=保持最后速度，随下一帧 cmd_vel 写入固件 SecurityLevel）
- **自动回充**：发布 `/robot_recharge_flag`（1 开 / 0 关，发布后自动补发一帧
  零速 cmd_vel 把标志位带给固件）；卡内显示固件确认的回充模式、回充红外、
  充电状态与电流。回充中由充电桩 CAN 设备引导底盘，手动遥控可打断，
  低压禁动豁免
- **RGB 灯带**：颜色选择器调 `/set_rgb_color` 服务（robot_interfaces/SetRgb，
  驱动转固件 `0x04` 串口帧）。固件优先级：充电指示 > 低电量 > 超声波警示 >
  用户自定义
- 参数调节：通过 `rcl_interfaces/GetParameters`/`SetParameters` 服务读写
  `/wheeltec_robot` 上的 `odom_x_scale`、`odom_y_scale`、
  `odom_z_scale_positive`、`odom_z_scale_negative`
- 折线图：电压、cmd_vel (vx / wz)
- **3D 视图（嵌入式 RViz 替代）**：原生 three.js + 浏览器端 TF（直接订阅
  `/tf`+`/tf_static`），图层：Grid、LaserScan(`/scan`)、Odometry 轨迹、
  **机器人模型（URDF）**、**SLAM 地图（/map）**。Fixed frame 默认
  `odom_combined`。机器人模型经 rosbridge 取 `robot_state_publisher` 的
  `robot_description`（xacro 已展开），浏览器自解析 link/visual
  （box/cylinder/sphere/STL/DAE），网格由 web_server 新增的
  `/pkg/<包名>/<路径>` 路由从 ament share 提供，每个 link 按实时 TF 摆放
  （无需关节运动学）；多个 robot_state_publisher（底盘+机械臂）可逗号并列。
  SLAM 地图复用 VLA 子页的 `/map` 数据铺为地面贴图，按 map→fixed TF 对齐，
  未定位时自动隐藏。
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
      （MoveJoint，二次确认），可一键填入当前关节角或预设位（观察位
      look / 零位 zero / arm_demo FK 演示位）。
    - 位姿运动 IK：末端 X/Y/Z + RPY（角度制，内部转四元数）经
      `/motion_service/move_joint|move_line`（cartesian 模式）下发，逆解
      由乐白控制器完成（不经 MoveIt、无碰撞检查，二次确认）；
      "IK 演示位"即 `arm_demo ik_demo` 的目标位姿。
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
