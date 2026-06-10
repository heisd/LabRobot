# yesense_interface

元生(YeSense)IMU 系列产品的 ROS2 自定义消息接口包,供 `yesense_std_ros2` 驱动节点发布解析后的传感器数据。

## 概述

纯接口包(无可执行节点)。涵盖原始 IMU、欧拉角、四元数、GNSS 位置、压力、温度等几乎所有 YeSense 协议字段,方便上层应用按需订阅。

## 目录结构

```
yesense_interface/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Tid.msg / SampleTimestamp.msg / Utc.msg
    ├── ThreeAxis.msg                # 三轴通用结构
    ├── SensorTemp.msg
    ├── ImuData.msg / ImuDataTenAxis.msg
    ├── EulerAngle.msg / EulerOnly.msg / Quat.msg
    ├── Pressure.msg
    ├── GnssPos.msg / PosOnly.msg / Vel.msg
    ├── NavStatus.msg / NavAll.msg / NavMin.msg / NavMinUtc.msg
    ├── AttitudeAllData.msg / AttitudeMinAhrs.msg / AttitudeMinVru.msg
    └── RobotLord.msg
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`、`builtin_interfaces`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 主要消息说明

### `ImuData.msg`

```
yesense_interface/Tid               tid
yesense_interface/ThreeAxis         acc       # 三轴加速度
yesense_interface/ThreeAxis         gyro      # 三轴陀螺
yesense_interface/SensorTemp        temp      # 温度
yesense_interface/SampleTimestamp   sample_timestamp
```

### `EulerAngle.msg`

```
float32 pitch
float32 roll
float32 yaw
```

### `Quat.msg`

```
float64 q0 / q1 / q2 / q3   # 四元数(w, x, y, z)
```

### `ImuDataTenAxis.msg`

包含加速度、陀螺、磁力计、温度等十轴信息(具体字段参考 .msg 源)。

### `GnssPos.msg` / `PosOnly.msg` / `NavAll.msg` / `NavMinUtc.msg`

GNSS/导航数据,经纬高、速度、UTC 时间等。

### `AttitudeAllData.msg` / `AttitudeMinAhrs.msg` / `AttitudeMinVru.msg`

完整 / AHRS 简版 / VRU 简版的姿态数据。

### `RobotLord.msg`

机器人主控扩展数据(根据厂商协议)。

## 编译

```bash
colcon build --packages-select yesense_interface
source install/setup.bash
ros2 interface show yesense_interface/msg/ImuData
ros2 interface show yesense_interface/msg/EulerAngle
```

## 注意事项

1. 多数消息为厂商私有协议定义,不需要直接使用,建议优先订阅 `sensor_msgs/Imu`(由 `yesense_std_ros2` 发布)。
2. 修改字段后,所有依赖包需要重新编译。
