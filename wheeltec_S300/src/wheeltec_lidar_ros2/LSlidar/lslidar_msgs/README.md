# lslidar_msgs

镭神(Leishen)激光雷达 ROS2 自定义消息接口包,供 `lslidar_driver` 与上层应用使用。

## 概述

仅接口包(无可执行节点),定义雷达原始包、单点、单线、整圈扫描和 DIFOP(设备信息)等数据结构。

## 目录结构

```
lslidar_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── LslidarPoint.msg     # 单点
    ├── LslidarScan.msg      # 单线 360° 扫描
    ├── LslidarPacket.msg    # 原始数据包(2000B)
    ├── LslidarDifop.msg     # DIFOP 设备信息
    └── LslidarSweep.msg     # 多线一帧 sweep
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`、`builtin_interfaces`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 消息定义

### `msg/LslidarPoint.msg`

```
float32 time          # 该点的时间戳(相对帧首)
float64 x, y, z       # 已转换到传感器坐标系的笛卡尔坐标
float64 azimuth       # 原始方位角
float64 distance      # 距离
float64 intensity     # 强度
```

### `msg/LslidarScan.msg`

```
float64 altitude         # 本扫描线的俯仰角
LslidarPoint[] points    # 0~359.99° 按方位角排序的有效点
```

### `msg/LslidarPacket.msg`

```
builtin_interfaces/Time stamp   # 数据包接收时间
uint8[2000] data                # 原始 2000 字节
int64 temperature               # 雷达温度
int64 rpm                       # 转速
```

### `msg/LslidarSweep.msg`

```
std_msgs/Header header
LslidarScan[16] scans   # 16 线雷达一帧 sweep
```

### `msg/LslidarDifop.msg`

设备配置/状态信息(具体字段以源文件为准)。

## 编译

```bash
colcon build --packages-select lslidar_msgs
source install/setup.bash
ros2 interface show lslidar_msgs/msg/LslidarSweep
```

## 注意事项

1. 上层应用通常只需要订阅标准 `sensor_msgs/LaserScan` 或 `sensor_msgs/PointCloud2`,本包定义的消息主要供 `lslidar_driver` 内部使用或用于调试。
2. 修改字段后,所有依赖包都需要重新编译。
