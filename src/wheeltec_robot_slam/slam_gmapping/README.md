# slam_gmapping (WheelTec 修改版)

WheelTec 机器人使用的 `slam_gmapping` ROS2 适配版本,基于经典 GMapping 粒子滤波 SLAM,完成 2D 激光雷达建图。

## 概述

该包是 ROS1 `slam_gmapping` 的 ROS2 移植/适配版本,底层依赖 `openslam_gmapping` 算法库。配合 WheelTec 底盘和雷达,即可启动建图。

## 目录结构

```
slam_gmapping/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   └── slam_gmapping.cpp       # 主节点(粒子滤波 SLAM)
└── launch/
    └── slam_gmapping.launch.py # 启动文件
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`nav_msgs`、`sensor_msgs`、`geometry_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`openslam_gmapping`、`message_filters`

## 节点说明

### `slam_gmapping`

订阅:
- `/scan` (sensor_msgs/LaserScan) — 激光雷达扫描数据
- TF: `odom -> base_link`(由底盘提供)

发布:
- `/map` (nav_msgs/OccupancyGrid) — 实时占据栅格地图
- `/map_metadata` (nav_msgs/MapMetaData)
- TF: `map -> odom`(SLAM 输出)

主要参数(默认值可在源码中查阅):
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `base_frame` | `base_link` | 机器人本体坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `map_update_interval` | `5.0` | 地图更新间隔(s) |
| `maxUrange` | `16.0` | 雷达最大有效距离 |
| `particles` | `30` | 粒子数 |
| `linearUpdate` / `angularUpdate` | `1.0` / `0.5` | 触发更新的线/角位移阈值 |
| `temporalUpdate` | `-1.0` | 时间更新阈值 |

## 启动文件

### `launch/slam_gmapping.launch.py`

包含 `turn_on_wheeltec_robot` 的底盘启动文件和雷达启动文件,然后启动 `slam_gmapping` 节点,实现"底盘 + 雷达 + GMapping"一体化建图。

参数:
- `use_sim_time` — 默认 `false`,若回放 bag 则需置 `true`。

## 编译与运行

```bash
colcon build --packages-up-to slam_gmapping
source install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

## 使用示例

```bash
# 1. 启动 GMapping 建图(已含底盘 + 雷达)
ros2 launch slam_gmapping slam_gmapping.launch.py

# 2. 启动 RViz 查看地图
ros2 launch wheeltec_rviz2 wheeltec_slam.launch.py

# 3. 用键盘遥控机器人扫一遍环境
ros2 run wheeltec_robot_keyboard wheeltec_keyboard

# 4. 保存地图
ros2 launch wheeltec_nav2 save_map.launch.py
```

## 注意事项

1. GMapping 是粒子滤波算法,对动态环境与大场景效果有限,推荐用于中小室内场景。
2. 粒子数越多越准但 CPU 消耗越高,树莓派类平台建议 30 左右。
3. 该包依赖 `openslam_gmapping`,必须一同编译。
4. 与其他 SLAM(cartographer / slam_toolbox)一次只能选一个运行。
