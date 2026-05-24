# wheeltec_cartographer

WheelTec 机器人对 Google `cartographer` 的封装包,提供针对 WheelTec 底盘和雷达调优的 lua 配置,以及一键启动 launch。

## 概述

Cartographer 是 Google 开源的 2D/3D 实时 SLAM 框架,基于图优化与子图构建,精度高、闭环效果好。本包将 cartographer 的 lua 配置文件、launch 文件与 WheelTec 底盘集成。

## 目录结构

```
wheeltec_cartographer/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── cartographer.lua             # WheelTec 标定后的 cartographer 主配置
└── launch/
    ├── cartographer.launch.py       # 主启动文件
    ├── realsense_d400.launch.py     # 配合 RealSense D400 系列(3D)
    ├── slam_rtabmap.launch.py       # 与 rtabmap 联用示例
    └── turtlebot3_scan.launch.py    # Turtlebot3 兼容示例
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `cartographer_ros`、`cartographer_ros_msgs`、`rclcpp`、`rclpy`、`nav_msgs`、`sensor_msgs`、`tf2`、`tf2_ros`、`turn_on_wheeltec_robot`

## 启动文件

### `cartographer.launch.py`

1. 启动 `turn_on_wheeltec_robot.launch.py`(传入 `carto_slam:=true`,使其使用 `ekf_carto.yaml`)。
2. 启动 `wheeltec_lidar.launch.py` 雷达。
3. 启动 `cartographer_ros/cartographer_node`,加载 `config/cartographer.lua`。
4. 启动 `cartographer_ros/cartographer_occupancy_grid_node`,生成 `/map` 占据栅格,默认分辨率 0.05、发布周期 0.5s。

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `use_sim_time` | `False` | 仿真模式 |
| `cartographer_config_dir` | 本包 `config/` | 配置目录 |
| `configuration_basename` | `cartographer.lua` | 配置文件名 |
| `resolution` | `0.05` | 栅格分辨率 |
| `publish_period_sec` | `0.5` | 栅格发布周期 |

### `cartographer.lua`

主要字段:
```lua
map_frame = "map"
tracking_frame = "base_footprint"
published_frame = "base_footprint"
odom_frame = "odom_combined"
use_odometry = true / false
publish_to_tf = true
publish_tracked_pose = true
```

(具体子图大小、IMU、扫描匹配等参数在 lua 中详细列出。)

### 其他 launch

- `realsense_d400.launch.py` — 配合 Intel RealSense D400 系列做 RGB-D 数据接入。
- `slam_rtabmap.launch.py` — 与 `rtabmap` 联合(2D 激光 + 视觉)。
- `turtlebot3_scan.launch.py` — Turtlebot3 兼容示例(可作模板)。

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_cartographer
source install/setup.bash
ros2 launch wheeltec_cartographer cartographer.launch.py
```

## 使用示例

```bash
# 启动建图(已含底盘 + 雷达)
ros2 launch wheeltec_cartographer cartographer.launch.py

# RViz 查看
ros2 launch wheeltec_rviz2 wheeltec_slam.launch.py

# 遥控走完后保存地图
ros2 run nav2_map_server map_saver_cli -f ~/carto_map
```

## 注意事项

1. `tracking_frame` 必须与底盘实际发布的 TF 一致,默认 `base_footprint`,与 `turn_on_wheeltec_robot` 配套。
2. `odom_frame = "odom_combined"` 必须与 EKF 输出一致,该参数已在 `turn_on_wheeltec_robot` 设为对应值。
3. 如果使用纯雷达 SLAM,可在 lua 中关闭 `use_imu`、`use_odometry` 中的部分项。
4. 闭环检测对算力敏感,树莓派类设备建议降低粒子/子图数量。
