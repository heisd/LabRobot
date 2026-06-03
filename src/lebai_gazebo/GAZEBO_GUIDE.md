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

## 七、在 Dashboard 里用

打开 Dashboard 顶部新增的 **仿真** 页签，点【Gazebo 仿真场景 + 机械臂】启动即可（绿点=运行中，
可看日志）。仿真任务与真机任务资源不冲突，但请勿同时连真机以免混淆。
