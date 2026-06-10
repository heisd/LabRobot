# astra_camera_msgs

奥比中光(Orbbec)Astra/Dabai/Gemini 系列深度相机的 ROS2 自定义消息与服务接口包。

## 概述

仅接口包,无可执行节点。供 `astra_camera` 驱动节点发布设备信息、外参、元数据,以及通过 ROS2 service 提供设备配置接口。

## 目录结构

```
astra_camera_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── DeviceInfo.msg        # 设备信息(序列号、固件版本等)
│   ├── Extrinsics.msg        # 外参(旋转、平移)
│   └── Metadata.msg          # 元数据(JSON)
└── srv/
    ├── GetCameraInfo.srv
    ├── GetCameraParams.srv
    ├── GetDeviceInfo.srv
    ├── GetInt32.srv
    ├── GetString.srv
    └── SetInt32.srv
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`、`sensor_msgs`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 消息定义

### `DeviceInfo.msg`

```
std_msgs/Header header
string name
int32 vid
int32 pid
string serial_number
string firmware_version
string supported_min_sdk_version
string hardware_version
```

### `Extrinsics.msg`

```
std_msgs/Header header
float64[9] rotation       # 3x3 旋转矩阵(行优先)
float64[3] translation    # 平移向量(m)
```

### `Metadata.msg`

```
std_msgs/Header header
string json_data
```

## 服务定义

### `GetCameraInfo.srv`

```
---
sensor_msgs/CameraInfo info
bool success
string message
```

### `GetCameraParams.srv`(双目内外参)

```
---
float32[4] l_intr_p     # 左目内参
float32[4] r_intr_p     # 右目内参
float32[9] r2l_r        # 右到左旋转
float32[3] r2l_t        # 右到左平移
float32[5] l_k          # 左目畸变
float32[5] r_k          # 右目畸变
bool success
string message
```

### `GetDeviceInfo.srv`

```
---
DeviceInfo info
bool success
string message
```

### `GetInt32.srv` / `GetString.srv` / `SetInt32.srv`

通用读 / 写 SDK 属性接口。

## 编译

```bash
colcon build --packages-select astra_camera_msgs
source install/setup.bash
ros2 interface show astra_camera_msgs/msg/DeviceInfo
ros2 interface show astra_camera_msgs/srv/GetCameraParams
```

## 注意事项

1. 该接口包同时被 `astra_camera` 驱动与上层应用使用,修改字段后请重新编译所有相关包。
2. 服务返回的 `success`、`message` 字段统一遵循 SDK 调用结果约定,出错时由 `message` 给出说明。
