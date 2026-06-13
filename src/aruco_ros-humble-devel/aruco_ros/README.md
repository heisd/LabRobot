# aruco_ros

ArUco 视觉标识的 ROS2 节点封装,基于 `aruco` 库实现单 / 双 Marker 检测以及多 Marker 数组发布。

## 概述

提供三类节点:
1. `single` — 检测单个指定 ID 的 Marker,发布位姿与 TF。
2. `double` — 同时检测两个指定 ID 的 Marker,常用于参考系/目标对的对齐场景。
3. `marker_publisher` — 检测画面中所有 Marker,发布 `aruco_msgs/MarkerArray`。

## 目录结构

```
aruco_ros/
├── CMakeLists.txt
├── package.xml
├── CHANGELOG.rst
├── include/aruco_ros/             # 公共头
├── src/
│   ├── simple_single.cpp          # 单 Marker 节点
│   ├── simple_double.cpp          # 双 Marker 节点
│   ├── marker_publish.cpp         # 全 Marker 节点
│   └── aruco_ros_utils.cpp        # 公用工具
├── cfg/                           # 动态参数配置
├── etc/                           # 样例 Marker、参数
└── launch/
    ├── single.launch.py
    ├── double.launch.py
    ├── marker_publisher.launch.py
    └── aruco_recognize.launch.py   # 综合示例
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`geometry_msgs`、`visualization_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`cv_bridge`、`image_transport`、`aruco`、`aruco_msgs`、`OpenCV`

## 节点说明

### `single`

订阅:
- `<image_topic>` — 相机图像
- `<camera_info_topic>` — 内参

发布:
- `~/pose` (geometry_msgs/PoseStamped) — Marker 位姿
- `~/transform` (geometry_msgs/TransformStamped)
- `~/position` (geometry_msgs/Vector3Stamped)
- `~/marker` (visualization_msgs/Marker) — RViz 可视化
- `~/pixel` (geometry_msgs/PointStamped) — 在图像中的像素坐标
- TF: `camera_frame -> marker_frame`(可选)

参数:
| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `marker_size` | `0.05` | Marker 实际尺寸(m) |
| `marker_id` | `300` | 待识别 ID |
| `reference_frame` | `""` | 输出参考系,空则使用 `camera_frame` |
| `camera_frame` | `""` | 相机坐标系 |
| `marker_frame` | `""` | Marker TF 名称 |
| `image_is_rectified` | `true` | 图像是否已矫正 |
| `min_marker_size` | `0.02` | 最小有效尺寸 |
| `detection_mode` | `""` | `DM_NORMAL` / `DM_FAST` / `DM_VIDEO_FAST` |

### `double`

类似 `single`,但同时检测两个 ID,常用于"参考 marker → 目标 marker"的相对位姿。

### `marker_publisher`

发布:
- `~/markers` (aruco_msgs/MarkerArray)

## 启动文件

- **`single.launch.py`** — 启动 `single` 节点。
- **`double.launch.py`** — 启动 `double` 节点。
- **`marker_publisher.launch.py`** — 启动 `marker_publisher`。
- **`aruco_recognize.launch.py`** — 综合示例(WheelTec 调整版),通常与相机驱动一起启动。

## 编译与运行

```bash
colcon build --packages-up-to aruco_ros
source install/setup.bash

# 单 marker 识别
ros2 launch aruco_ros single.launch.py marker_size:=0.05 marker_id:=100
```

## 使用示例

```bash
# 1. 启动相机
ros2 launch usb_cam usb_cam_launch.py

# 2. 启动 aruco 节点(注意把 image / camera_info 重映射到实际相机话题)
ros2 launch aruco_ros marker_publisher.launch.py

# 3. RViz 查看 /marker 或 /markers
```

## 注意事项

1. `marker_size` 必须与实际打印 Marker 的边长一致,否则估计的距离会成比例错误。
2. 若图像没有矫正,务必设置 `image_is_rectified:=false`。
3. `reference_frame` 与 `camera_frame` 必须与 TF 中实际存在的 frame 一致。
4. 多 Marker 检测对算力敏感,建议在 USB3 相机 + Jetson 上使用 `DM_NORMAL`。

## 构建失败 应该是Opencv版本的问题
```bash
> colcon build --packages-select aruco_ros
Starting >>> aruco_ros
--- stderr: aruco_ros                           
CMake Error at CMakeLists.txt:13 (find_package):
  Could not find a configuration file for package "OpenCV" that is compatible
  with requested version "4.10".

  The following configuration files were considered but not accepted:

    /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake, version: 4.5.4
    /lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake, version: 4.5.4



---
Failed   <<< aruco_ros [11.4s, exited with code 1]

```
