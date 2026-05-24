# orb_slam2_ros (WheelTec ROS2 适配)

ORB-SLAM2 的 ROS2 节点封装,支持单目 / 双目 / RGB-D 三种相机输入,实现基于特征点的视觉 SLAM。

## 概述

ORB-SLAM2 是稀疏特征点视觉 SLAM 的代表性算法。本包将其封装为 ROS2 节点,直接订阅 ROS2 话题(图像、相机内参),发布相机位姿、地图点、轨迹等话题,适合用于 WheelTec 机器人的视觉建图与定位。

## 目录结构

```
orb_slam_2_ros-ros2/
├── CMakeLists.txt
├── package.xml
├── README.md / LICENSE.txt / License-gpl.txt
├── Dependencies.md
├── orb_slam2/              # ORB-SLAM2 核心库源码与第三方依赖(g2o、DBoW2、Pangolin 可选)
├── ros/                    # ROS2 节点封装
├── docker/                 # Docker 部署示例
├── srv/                    # 自定义服务(地图保存 / 加载)
└── rviz_config.rviz        # 推荐 RViz 配置
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`cv_bridge`、`image_transport`、`message_filters`、`OpenCV`、`Pangolin`、`Eigen3`、`PCL`

## 节点说明

提供三个独立可执行文件,分别对应三种相机输入:

### `mono`(单目)

订阅:
- `/camera/image_raw` (sensor_msgs/Image)

### `stereo`(双目)

订阅:
- `/camera/left/image_raw`、`/camera/right/image_raw`(sensor_msgs/Image)

### `rgbd`(RGB-D)

订阅:
- `/camera/rgb/image_raw`、`/camera/depth/image_raw`(sensor_msgs/Image)

公共发布:
- `~/map_points` (sensor_msgs/PointCloud2) — 实时稀疏地图点
- `~/pose` (geometry_msgs/PoseStamped) — 相机位姿
- TF: `map -> camera_link`

主要参数:
| 参数 | 含义 |
| --- | --- |
| `voc_file` | ORB 词袋文件路径(`ORBvoc.txt`) |
| `settings_file` | 相机内参 yaml |
| `publish_pointcloud` | 是否发布点云 |
| `publish_pose` | 是否发布位姿 |
| `localize_only` | 仅定位模式 |
| `reset_map` | 启动时重置已加载的地图 |
| `load_map` / `map_file` | 启动时加载序列化地图 |
| `pointcloud_frame_id` / `camera_frame_id` |

## 服务定义

`srv/` 内含地图加载/保存服务,例如 `SaveMap.srv`、`LoadMap.srv`,通过 `ros2 service call` 触发。

## 编译与运行

```bash
# 第一次编译需先下载 ORBvoc.txt 词袋(通常放到 orb_slam2/Vocabulary/)
colcon build --packages-up-to orb_slam2_ros
source install/setup.bash

# 单目 SLAM 示例
ros2 run orb_slam2_ros mono \
    --ros-args -p voc_file:=$HOME/ORBvoc.txt \
               -p settings_file:=$HOME/camera_mono.yaml
```

## 使用示例

```bash
# 1. 启动 USB 摄像头/Astra 相机驱动
ros2 launch usb_cam usb_cam.launch.py

# 2. 启动 ORB-SLAM2 单目节点
ros2 run orb_slam2_ros mono --ros-args -p voc_file:=... -p settings_file:=...

# 3. RViz 加载 rviz_config.rviz 查看地图与轨迹
```

## 注意事项

1. 必须提前下载 `ORBvoc.txt` 词袋(可从 ORB-SLAM2 官方仓库获取)。
2. `settings_file` 必须按 ORB-SLAM2 官方格式书写相机内参/畸变/帧率/特征点参数。
3. 单目 SLAM 存在尺度不确定性,与 Nav2 联用需要额外尺度对齐。
4. Pangolin 可视化默认开启,如部署到无显示器环境需要关闭。
5. 大场景或快速运动会导致跟踪丢失,需要做闭环或多线程优化。
