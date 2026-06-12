# Gazebo 仿真使用说明（lebai_gazebo）

给乐白 LM3 机械臂提供 **Gazebo Classic（11）仿真场景**：地面 + 桌子 + 标准物体 + 机械臂，
可在仿真里运行抓取/视觉/VLM 等功能，并用 Gazebo GUI 的 Insert 面板继续导入标准物体。

> 状态说明：本仓库原先**没有任何仿真接口**（`demo.launch.py` 是 MoveIt 原装 Panda 演示，
> 机器人 xacro 也没有 gazebo/ros2_control 标签）。本包是从零新增的仿真脚手架。
> 由于开发环境无 Gazebo/ROS，**这些文件未能在线编译/运行验证**，首次在真机/工作站上跑时
> 可能需要按下文的"需要微调的地方"调整。

## 一、包含什么

| 文件 | 作用 |
|------|------|
| `urdf/lm3_gazebo.xacro` | 机械臂 + 固定到 world + ros2_control(GazeboSystem) + 末端深度相机 |
| `config/gazebo_controllers.yaml` | `joint_state_broadcaster` + `lebai_trajectory_controller`（名字与 MoveIt 一致） |
| `worlds/grab_world.world` | 地面/阳光/桌子/可乐罐/木块/啤酒（`model://` 标准模型） |
| `launch/gazebo.launch.py` | 启动 Gazebo GUI + 场景 + 生成机械臂 + 激活控制器 |

## 二、依赖（在运行的机器上装）

Gazebo Classic 11 + ros2_control 的 Gazebo 插件：

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-xacro
```

## 三、编译 & 运行

```bash
cd ~/lebai
colcon build --packages-select lebai_gazebo
source install/setup.bash
ros2 launch lebai_gazebo gazebo.launch.py
```

### 按抓取功能选择世界（文件名即功能）

`gazebo.launch.py` 与 `gazebo_grab.launch.py` 都支持 `world` 参数（默认 `grab_world`），
按要验证的抓取方案切换：

| 抓取功能 | world | 场内物体 |
|---|---|---|
| 通用 | `grab_world` | 可乐罐 / 木块 / 啤酒 |
| HSV 颜色抓取 | `grab_hsv_color` | 红/绿色块 + 红可乐罐（默认抓红色，改 HSV 抓绿色）|
| YOLO 抓取 | `grab_yolo` | 啤酒(bottle)/杯子(cup)/碗(bowl)/可乐罐 |
| KCF 跟踪抓取 | `grab_kcf` | 一个主目标 + 一个干扰物（画面拖框跟踪）|
| ArUco 抓取 | `grab_aruco` | ArUco 标记牌（`model://aruco_marker`）+ 木块 |
| VLM 语言抓取 | `grab_vlm` | 可乐罐/啤酒/碗/杯子/锤子（自然语言选物）|

```bash
ros2 launch lebai_gazebo gazebo_grab.launch.py world:=grab_yolo
```

> **ArUco 世界首次用前**：纹理默认是占位图，先生成可识别标记（需 opencv-contrib）：
> ```bash
> python3 src/lebai_gazebo/scripts/make_aruco_marker.py        # 默认 DICT_6X6_250 id 0
> ```
> 生成的图会覆盖 `models/aruco_marker/materials/textures/aruco_marker.png`；若识别不到，
> 按你们 `aruco_node` 的实际字典改 `--dict/--id` 重新生成。`model://aruco_marker` 由本包
> 的 `GAZEBO_MODEL_PATH` 环境钩子解析（**需先 colcon build 并 source install/setup.bash**）。

> **在 Dashboard 一键启动**：dashboard launch 起了 `sim_launcher` 节点，在面板"机械臂仿真"页
> 选好世界点"启动仿真"，等价后端执行上面的 `ros2 launch …`（详见 wheeltec_dashboard）。

会打开 Gazebo GUI，自动加载场景和机械臂。控制器起来后可以验证：

```bash
ros2 control list_controllers          # 应看到 active 的两个控制器
ros2 topic list | grep camera_arm      # 仿真相机话题
ros2 topic echo /joint_states          # 关节状态
```

## 四、用 Gazebo GUI 导入物体 / 模型

- 左侧 **Insert** 面板：列出本地与 Fuel 在线模型库的标准模型，直接拖进场景即可
  （如 cup、bowl、table、coke_can 等）。首次拖入在线模型会自动下载。
- 想把**自己的机械臂/物体模型**也出现在 Insert 列表里：把模型目录放到
  `~/.gazebo/models/<your_model>/`（含 `model.config` + `model.sdf`），重启 Gazebo 即可。
- `worlds/grab_world.world` 里已 `include` 了几个标准物体作为初始场景，可自行增删。

## 五、在仿真里跑我们的功能

仿真相机已发布 `/camera_arm/color/image_raw`、`/camera_arm/depth/image_raw` 等，
理论上 HSV/YOLO/KCF/VLM 节点可直接连上。建议加 `use_sim_time:=true`。

- MoveIt 规划：仿真的控制器名 `lebai_trajectory_controller` 与
  `lebai_lm3_moveit_config` 的 `lm3_controllers.yaml` 一致，因此 `move_group`
  可直接驱动仿真机械臂（启动 move_group 时不要再起真机 `robot_interface`）。

### 一键端到端抓取（gazebo_grab.launch.py）

把 Gazebo + 相机 + MoveIt + HSV 视觉 + 抓取服务串成一条链路：

```bash
ros2 launch lebai_gazebo gazebo_grab.launch.py
```

它做了：
- 复用 `gazebo.launch.py`（场景 + 机械臂 + 控制器 + 相机）；
- `move_group`（仿真参数，`use_sim_time`，控制器 `lebai_trajectory_controller`）；
- `world→base_link` 恒等静态 TF（grab_service 以 base_link 为参考系）；
- HSV 视觉节点（默认红色阈值正好识别桌上的**可乐罐**，内参指向仿真相机 `/camera_arm/color/camera_info`）；
- `grab_service_node`。

触发抓取（识别到目标后，target_frame 已在发布）：

```bash
ros2 service call /obj_grab_service grab_demo/srv/GrabObject "{obj_link: 'target_frame'}"
```

> 启动后到 Dashboard 顶部"仿真 (Gazebo)"分组的 **机械臂仿真** 页看转发的 Gazebo 场景，
> 在"监控与抓取"页控制机械臂、看目标距离。

### 端到端首次跑可能要调的地方（除第六节外）

- **控制器 action 命名**：`lm3_controllers.yaml` 里 `action_ns: ""` 是给真机轨迹动作服务器用的；
  ros2_control 的 `joint_trajectory_controller` 动作在
  `/lebai_trajectory_controller/follow_joint_trajectory`。若 MoveIt 执行时报找不到 action，
  把 MoveIt 控制器配置的 `action_ns` 改成 `follow_joint_trajectory`。
- **base_link 与机械臂基座**：仿真里 `base_link`(=world 原点) 与 `lebai_base_link`(桌面 z=1.015) 通过
  world 关联；若抓取目标位姿明显偏移，检查 `world_to_base` 的 z 和 `world→base_link` 是否一致。
- **相机帧**：vision 用 `camera_arm_depth_optical_frame`（由 gazebo 相机插件 + xacro 的光学关节提供），
  与真机的 `camera_link`/手眼标定不同，纯仿真用这套即可。

## 六、需要微调的地方（首次运行重点检查）

1. **相机安装位姿**：`lm3_gazebo.xacro` 里 `tool0_to_camera` 的 `origin` 是估计值，
   请按你们的手眼标定结果调整，否则仿真里抓取定位会偏。
2. **相机内参**：仿真相机自带 `camera_info`，但视觉节点默认订阅 `/gemini_info`
   （`camera_info_node` 发布的是真机内参）。仿真时建议把视觉节点的
   `camera_info_topic` 指到 `/camera_arm/color/camera_info`，或让 `camera_info_node`
   不启动。
3. **桌面高度 / 底座高度**：`world_to_base` 的 z（1.015）与 world 里桌子高度需对应，
   不同 table 模型高度可能不同。
4. **Gazebo 版本**：本包按 Gazebo Classic 11 写。如果你们用的是 Ignition/Gazebo(gz-sim)，
   插件名（`libgazebo_ros2_control.so` / `libgazebo_ros_camera.so`）和 world 语法需要改用
   `ros_gz` 对应版本。
5. **物理稳定性**：机械臂 6 个运动连杆有 inertial；base/tool0/gripper 等是占位 link，
   已用 fixed joint 固定到 world，一般稳定。若出现抖动，检查 collision mesh 与质量。

## 七、在 Dashboard 里看（Gazebo 场景转发到 Web）

`grab_world.world` 里加了一个俯视**场景相机**，把整个抓取仿真画面发布成 ROS 图像话题
`/sim_scene/arm/image_raw`；Dashboard 的 `web_video_server` 把它转成 MJPEG，在顶部
**仿真 (Gazebo)** 分组的 **机械臂仿真** 页直接显示——机器人/工作站不开 Gazebo GUI 也能
在网页里看仿真场景。该页同时显示眼在手相机 `/camera_arm/color/image_raw` 与 HSV 识别调试图
`/color_node/detection_image`，并实时显示抓取目标距离。

用法：
1. 在能跑 Gazebo 的机器上 `ros2 launch lebai_gazebo gazebo_grab.launch.py`（或 `gazebo.launch.py`）；
2. 在同一工作空间起 Dashboard（`ros2 launch wheeltec_dashboard dashboard.launch.py`，自带
   rosbridge + web_video_server）；
3. 浏览器打开 Dashboard → **机械臂仿真** 页看场景，**监控与抓取** 页控制机械臂。

> 面板只做"场景转发 + 控制"，不在网页里直接拉起 Gazebo 进程（启动仍在终端 `ros2 launch`）。
> 仿真任务与真机任务资源不冲突，但请勿同时连真机以免混淆。Wheeltec 底盘仿真见
> `wheeltec_gazebo` 包与面板的 **机器人底盘仿真** 页。
