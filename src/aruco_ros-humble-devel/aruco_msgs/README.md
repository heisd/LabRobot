# aruco_msgs

ArUco 视觉标识的 ROS2 消息接口包,供 `aruco_ros` 发布检测到的 ArUco Marker 位姿。

## 概述

纯接口包(无可执行节点)。定义单个 Marker 与 Marker 数组消息,通常被 RViz、`aruco_ros` 节点、上层应用一同使用。

## 目录结构

```
aruco_msgs/
├── CMakeLists.txt
├── package.xml
├── CHANGELOG.rst
└── msg/
    ├── Marker.msg
    └── MarkerArray.msg
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`、`geometry_msgs`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 消息定义

### `msg/Marker.msg`

```
std_msgs/Header                   header
uint32                            id            # ArUco ID
geometry_msgs/PoseWithCovariance  pose          # 含协方差的位姿
float64                           confidence    # 置信度
```

### `msg/MarkerArray.msg`

```
std_msgs/Header        header
aruco_msgs/Marker[]    markers
```

## 编译

```bash
colcon build --packages-select aruco_msgs
source install/setup.bash
ros2 interface show aruco_msgs/msg/Marker
```

## 注意事项

1. `id` 字段必须与所使用的 ArUco 字典一致(默认为 `ARUCO_MIP_36h12`)。
2. `pose` 是相对于相机或 `reference_frame`(由检测节点参数指定)的位姿。
