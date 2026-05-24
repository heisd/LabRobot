# wheeltec_slam_toolbox

WheelTec 机器人对 `slam_toolbox` 的封装包,提供同步/异步两种在线建图启动文件以及对应参数。

## 概述

`slam_toolbox` 是 ROS2 主流的 2D 激光 SLAM 框架,具备闭环检测、地图续建、序列化保存等能力。本包在其基础上集成了底盘和雷达 launch,实现一键建图。

## 目录结构

```
wheeltec_slam_toolbox/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── mapper_params_online_async.yaml   # 异步建图参数
│   └── mapper_params_online_sync.yaml    # 同步建图参数
├── launch/
│   ├── online_async_launch.py            # 异步建图启动
│   └── online_sync.launch.py             # 同步建图启动
└── src/
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`slam_toolbox`、`turn_on_wheeltec_robot`

## 启动文件

### `online_async_launch.py`

依次启动:
1. `turn_on_wheeltec_robot.launch.py` — 底盘
2. `wheeltec_lidar.launch.py` — 雷达
3. `slam_toolbox/async_slam_toolbox_node`,使用 `config/mapper_params_online_async.yaml`

### `online_sync.launch.py`

类似 async 版本,但启动 `sync_slam_toolbox_node`,使用 `mapper_params_online_sync.yaml`。同步模式精度更高、CPU 占用更高,适合精细建图。

## 参数配置

两个 yaml 内常见参数:

| 参数 | 含义 |
| --- | --- |
| `solver_plugin` | 后端求解器(Ceres/G2O) |
| `ceres_*` | Ceres 求解器配置 |
| `odom_frame` / `map_frame` / `base_frame` | TF 坐标系 |
| `scan_topic` | 默认 `/scan` |
| `mode` | `mapping` 或 `localization` |
| `resolution` | 地图分辨率 |
| `max_laser_range` | 雷达最大测距 |
| `minimum_time_interval` | 关键帧间隔 |
| `loop_search_maximum_distance` | 闭环检测距离 |
| `do_loop_closing` | 是否启用闭环 |

## 编译与运行

```bash
colcon build --packages-select wheeltec_slam_toolbox
source install/setup.bash

# 异步建图(推荐)
ros2 launch wheeltec_slam_toolbox online_async_launch.py

# 同步建图
ros2 launch wheeltec_slam_toolbox online_sync.launch.py
```

## 使用示例

```bash
# 建图
ros2 launch wheeltec_slam_toolbox online_async_launch.py
ros2 launch wheeltec_rviz2 wheeltec_slam.launch.py

# 遥控走完
ros2 run wheeltec_robot_keyboard wheeltec_keyboard

# 保存地图为 *.pgm + *.yaml
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# 也可在 RViz 上调用 slam_toolbox 提供的 "Serialize Map" 保存为 *.posegraph + *.data
```

## 注意事项

1. 异步模式适合实时性优先,同步模式适合精度优先。
2. 默认 `scan_topic` 为 `/scan`,若雷达 launch 重命名了话题需修改参数。
3. 续建/纯定位模式需把 yaml 中 `mode` 改为 `localization`,并通过参数 `map_file_name` 加载序列化地图。
4. 与 GMapping、Cartographer 等其他 SLAM 互斥,一次只能启动一个。
