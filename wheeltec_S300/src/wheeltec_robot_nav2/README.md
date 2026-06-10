# wheeltec_nav2

WheelTec S300/Mini/Pro 系列机器人的 Nav2 导航集成包,封装地图、Nav2 参数、RViz 配置以及一键启动脚本。

## 概述

该包是机器人导航的"上层入口"。它会同时启动底盘驱动、激光雷达、Nav2 协议栈、Waypoint Cycle、超声波避障(可选)等模块,加载预先建好的地图开始定位与导航。

## 目录结构

```
wheeltec_robot_nav2/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── wheeltec_nav2.launch.py        # 顶层启动:底盘+雷达+Nav2+Waypoint
│   ├── bringup_launch.py              # 基于 Nav2 官方 bringup 改写
│   └── save_map.launch.py             # 保存当前地图
├── param/
│   ├── nav_param_s300.yaml            # 通用 S300 Nav2 参数
│   ├── nav_param_s300_mini.yaml       # S300 Mini 参数
│   ├── nav_param_s300_pro.yaml        # S300 Pro 参数
│   └── back_up.yaml                   # 备份
├── map/
│   ├── WHEELTEC.pgm                   # 默认栅格地图
│   └── WHEELTEC.yaml                  # 地图元数据
└── rviz/                              # RViz 显示配置
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`nav2_bringup`、`nav2_msgs`、`nav2_common`、`launch_ros`、`yaml_cpp_vendor`、`xacro` 等
- 间接依赖: `turn_on_wheeltec_robot`、`wheeltec_ultrasonic`、`nav2_waypoint_cycle`

## 启动文件

- **`wheeltec_nav2.launch.py`** — 顶层启动文件:
  1. 读取环境变量 `ROBOT_TYPE`(默认 `s300_pro`),选择对应 `nav_param_<type>.yaml`。
  2. 内部硬编码 `ENABLE_ULTRASONIC = True/False` 控制是否启用超声波避障。如启用,自动在原始参数文件基础上生成带超声波 layer 的临时参数文件。
  3. 包含 `turn_on_wheeltec_robot.launch.py`(底盘 + EKF)。
  4. 包含 `wheeltec_lidar.launch.py`(雷达驱动)。
  5. 包含 `bringup_launch.py`(Nav2 协议栈)。
  6. 启动 `nav2_waypoint_cycle` 节点(支持多点循环导航)。
  7. 如启用超声波,启动 `supersonic+converter.launch.py`。
- **`bringup_launch.py`** — 来自 Nav2 官方 `bringup_launch.py` 的本地化版本,负责 amcl、planner_server、controller_server、recoveries、bt_navigator 等节点的一体化启动。可配置 `map`、`params_file`、`use_sim_time` 等参数。
- **`save_map.launch.py`** — 调用 `nav2_map_server/map_saver_cli` 保存当前 SLAM 建好的地图到 `map/` 目录。

## 参数配置

各 `nav_param_*.yaml` 是 Nav2 完整参数文件,涵盖:
- `amcl`:初始位姿、激光模型、粒子数等
- `bt_navigator`:行为树配置
- `controller_server` / `FollowPath`:DWB 或 RPP 控制器
- `planner_server` / `GridBased`:NavFn/Smac 全局规划
- `local_costmap` / `global_costmap`:代价地图层(static、obstacle、inflation、ultrasonic 可选)
- `recoveries_server`:Spin / BackUp 行为
- `waypoint_follower`:多点导航
- `robot_radius`、`inflation_radius` 等机器人尺寸相关字段已按机型预设

> 不同车型(s300 / s300_mini / s300_pro)的 `robot_radius`、`min_obstacle_height`、`scan_topic`、`max_vel` 等关键参数会有差异。

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_nav2
source install/setup.bash

export ROBOT_TYPE=s300_pro
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
```

## 使用示例

```bash
# 加载默认地图导航
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

# 使用自定义地图
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py map:=/path/to/my_map.yaml

# 在 RViz 中点击 2D Pose Estimate 设定初始位姿,然后点击 2D Goal Pose 发送导航目标
# 或在 RViz 上多次 Publish Point 设定途经点,由 nav2_waypoint_cycle 自动巡航

# 建图后保存
ros2 launch wheeltec_nav2 save_map.launch.py
```

## 注意事项

1. **必须**先设置环境变量 `ROBOT_TYPE`,否则会用默认参数,可能与实际机器人不符。
2. 超声波避障开关需要修改 `wheeltec_nav2.launch.py` 顶部 `ENABLE_ULTRASONIC` 常量后重新编译。
3. 默认地图 `WHEELTEC.pgm` 仅供示例,请用自己建好的地图替换。
4. 启动前确保底盘已上电,串口、激光雷达 udev 软链接已生成。
5. 给定的初始位姿过偏会导致 AMCL 无法收敛,建议用 RViz `2D Pose Estimate` 校准。
