# wheeltec_gazebo

Wheeltec 移动底盘的 **Gazebo Classic 11** 仿真：一台自洽的差速底盘 +
**按功能分的多个世界**，并把仿真场景经 `web_video_server` 转发到 Dashboard 的
**机器人底盘仿真** 页。

> 状态：开发环境无 Gazebo/ROS，这些文件**未能在线编译/运行验证**，是仿真脚手架；
> 首次在工作站/真机上跑可能要按"需要微调"一节调整。与 `lebai_gazebo`(机械臂仿真)同款风格。

## 一、包含什么

| 文件 | 作用 |
|------|------|
| `urdf/wheeltec_gazebo.xacro` | Wheeltec 差速底盘：车体 + 左右驱动轮 + 万向轮 + 2D 雷达 + 前向相机；带 diff_drive / ray / camera 三个 gazebo_ros 插件 |
| `worlds/wheeltec_slam_nav.world` | **建图 / 导航 / 避障**：grey_wall 围成的房间 + 书架/柜子/桌子/垃圾桶/锥桶/行人 |
| `worlds/wheeltec_rrt_explore.world` | **RRT 自主探索 / 迷宫**：外圈墙 + NIST 迷宫墙隔出走廊与未知区域 |
| `worlds/wheeltec_line_follow.world` | **巡线**：地面红色闭环赛道(4×3m，纯视觉) + 起点锥桶 |
| `worlds/wheeltec_target_follow.world` | **目标跟随 / 检测**：行人(站/走) + 物体，供 KCF/YOLO/骨架跟随与 YOLO 检测 |
| `worlds/wheeltec_vla_nav.world` | **VLA 语音导航 / 路径跟随**：分区房间 + 书架/餐桌/柜子/回充/访客等可命名地标 |
| `launch/gazebo.launch.py` | 启动 Gazebo + 选定世界 + 生成底盘 |

**世界文件名都带功能名**，一看即知用途。所有障碍物来自本机 `~/.gazebo/models`
标准模型，URI 用与 `lebai_gazebo/worlds/grab_world.world` 一致的绝对路径
`model:///home/user/.gazebo/models/XXX`，无需联网下载。

> 若你的 home 不是 `/home/user`（如 `/home/li`），把各 `.world` 里的 `/home/user/`
> 批量替换成你的实际家目录即可（grab_world.world 也是这个约定）。

## 二、依赖

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher
```

## 三、编译 & 运行

```bash
cd ~/LabRobot
colcon build --packages-select wheeltec_gazebo
source install/setup.bash

# 默认建图/导航世界
ros2 launch wheeltec_gazebo gazebo.launch.py

# 按功能切换世界(world 参数 = 文件名，不带 .world)
ros2 launch wheeltec_gazebo gazebo.launch.py world:=wheeltec_line_follow
ros2 launch wheeltec_gazebo gazebo.launch.py world:=wheeltec_rrt_explore x:=-3.0 y:=-3.0
ros2 launch wheeltec_gazebo gazebo.launch.py world:=wheeltec_target_follow
ros2 launch wheeltec_gazebo gazebo.launch.py world:=wheeltec_vla_nav
```

起来后验证：

```bash
ros2 topic echo /scan --once          # 2D 雷达
ros2 topic echo /odom --once          # 里程计
ros2 topic list | grep camera         # /camera/color/image_raw
ros2 topic list | grep sim_scene      # /sim_scene/chassis/image_raw(转发到 Web 的场景相机)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"  # 前进
```

## 四、和 Dashboard / 现有功能怎么搭

底盘话题特意和本仓库现有话题对齐，所以仿真起来后**现有面板和节点基本可直接用**：

- **遥控**：Dashboard"底盘控制"页或键盘节点发 `/cmd_vel` → diff_drive 驱动仿真底盘。
- **3D 视图**：能看 `/scan` 与 `/odom` 轨迹。注意仿真里程计坐标系是 `odom`
  （真机是 EKF 的 `odom_combined`），所以 3D 视图的 *Fixed frame* 在仿真时填 `odom`。
- **机器人底盘仿真页**：场景相机话题 `/sim_scene/chassis/image_raw` 经 web_video_server
  转 MJPEG 显示整个 Gazebo 画面；车上相机 `/camera/color/image_raw` 也可在该页/总览页看。
- **建图**：`world:=wheeltec_slam_nav` 或 `wheeltec_rrt_explore`，再起 SLAM
  （GMapping/Cartographer/Toolbox）；建图页地图实时刷新。
- **RRT 探索**：`world:=wheeltec_rrt_explore`，起 rrt_exploration，在建图页画边界点。
- **巡线**：`world:=wheeltec_line_follow`，起 `simple_follower_ros2`，相机俯看红线循迹。
- **跟随 / 检测**：`world:=wheeltec_target_follow`，起 KCF/YOLO/bodyreader 跟随场内行人。
- **VLA 语音导航 / 路径跟随**：`world:=wheeltec_vla_nav`，起 `vla_navigation` 的 vla_navigator，
  对房间里的书架/餐桌/柜子/回充区等地标做自然语言导航；路径录制/回放(wheeltec_path_follow)同此世界。

建议各功能节点都加 `use_sim_time:=true`。

## 五、需要微调的地方（首次运行重点检查）

1. **轮径 / 轮距 / 车体尺寸**：`wheeltec_gazebo.xacro` 顶部的 `wheel_radius` /
   `wheel_separation` / `body_*` 是按 S300 估的近似值，按你们实测调，否则里程计标定会偏。
2. **相机俯角**：`camera_joint` 的 pitch=0.45rad 是为看巡线估的，巡线看不到线就调这个角度/高度。
3. **雷达高度**：`laser` 在车体顶面上方 2cm；若被车体或物体遮挡，抬高 `laser_joint` 的 z。
4. **Gazebo 版本**：本包按 Gazebo **Classic 11** 写（插件 `libgazebo_ros_diff_drive.so` /
   `libgazebo_ros_ray_sensor.so` / `libgazebo_ros_camera.so`）。若用 Ignition/gz-sim，
   插件名与 world 语法需改用 `ros_gz` 对应版本。
5. **模型路径**：见第一节的 home 路径说明；模型不在 `~/.gazebo/models` 时会加载失败。
6. **更逼真外观**：想让车体像真车，把 `base_link` 的 `<box>` 视觉换成
   `package://rm_description/meshes/rm_eco65_arm/s300_pro_base_link.STL`（仅视觉，碰撞仍可留 box）。
