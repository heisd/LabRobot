# wheeltec_robot_rtab

WheelTec 机器人对 `rtabmap_ros` 的封装包,提供基于 RGB-D 相机或多传感器融合的视觉 SLAM、定位与 Nav2 集成。

## 概述

`rtabmap`(Real-Time Appearance-Based Mapping)是经典的开源 RGB-D / 视觉 SLAM 框架,支持 RGB-D、立体相机、3D 激光、2D 激光等多种输入,具备闭环检测、全局优化和地图持久化能力。本包提供 WheelTec 机器人的常用启动模板。

## 目录结构

```
wheeltec_robot_rtab/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── rtabmap.launch.py              # RGB-D + ICP odometry 主建图 launch
│   ├── rtabmap_localization.launch.py # 纯定位模式
│   ├── wheeltec_slam_rtab.launch.py   # 底盘 + 雷达 + rtabmap SLAM
│   └── wheeltec_nav2_rtab.launch.py   # rtabmap + Nav2 完整导航栈
└── params/
    └── rtabmap_nav_params.yaml        # Nav2 参数(rtabmap 适配)
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`rclpy`、`rtabmap_ros`、`rtabmap_sync`、`rtabmap_odom`、`rtabmap_slam`、`rtabmap_viz`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`tf2`、`tf2_ros`、`nav2_bringup`

## 启动文件

### `rtabmap.launch.py`

启动 rtabmap 主链:
1. `rtabmap_sync/rgbd_sync` — RGB + Depth + CameraInfo 时间同步。
2. `rtabmap_odom/icp_odometry` — 基于 ICP 的里程计(可作为视觉里程计的替代)。
3. `rtabmap_slam/rtabmap` — SLAM/定位主节点(根据 `localization` 参数二选一)。

关键参数(可在 launch 命令行传入):
| 参数 | 含义 |
| --- | --- |
| `localization` | `true`=纯定位,`false`=建图 |
| `use_sim_time` | 仿真时间 |
| `rgb_topic` / `depth_topic` / `camera_info_topic` | RGB-D 输入话题 |
| `qos` | QoS 配置(默认 1 或 2) |
| `rtabmap_viz` | 是否开启 rtabmap 可视化 GUI |

### `rtabmap_localization.launch.py`

直接用建好的 `.db` 数据库进入纯定位模式。

### `wheeltec_slam_rtab.launch.py`

启动底盘 + 雷达 + 相机 + rtabmap SLAM,适合一键建图。

### `wheeltec_nav2_rtab.launch.py`

将 rtabmap 的占据栅格作为 Nav2 输入,启动完整导航栈。`params/rtabmap_nav_params.yaml` 是 Nav2 的参数。

## 参数配置

`params/rtabmap_nav_params.yaml` 内容与 `wheeltec_nav2/param/nav_param_*.yaml` 类似,主要差异在:
- `controller_server` 与 `local_costmap` 针对 rtabmap 提供的 `/map` 做了调整。
- 加入 `voxel_layer`/`obstacle_layer` 处理 3D 点云。

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_robot_rtab
source install/setup.bash

# 建图模式
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py

# 定位 + 导航模式
ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py
```

## 使用示例

```bash
# 1. 建图(默认数据库 ~/.ros/rtabmap.db)
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py

# 2. 把机器人在场景中走一圈,rtabmap 会自动闭环

# 3. 切换为定位 + 导航
ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py localization:=true
```

## 注意事项

1. RGB-D 相机必须输出对齐良好的 RGB / Depth / CameraInfo,可用 `astra_camera` 或 RealSense。
2. 首次建图会在 `~/.ros/rtabmap.db` 生成数据库,要重新建图需先删除或加 `--delete_db_on_start`。
3. ICP odometry 对算力要求较高,弱平台建议关闭并使用底盘 odom。
4. 与其他 SLAM 互斥,一次只能启动一种。
5. Nav2 与 rtabmap 联用时务必保证两者的 `map_frame`、`odom_frame`、`base_frame` 一致。
