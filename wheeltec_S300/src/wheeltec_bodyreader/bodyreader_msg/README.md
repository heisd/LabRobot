# bodyreader_msg

WheelTec `bodyreader` 人体骨骼识别节点的 ROS2 消息接口包。

## 概述

纯接口包(无可执行节点)。定义人体骨骼关键点(19 个 Joint)、姿态、Mask、二维 / 三维向量等数据结构。

## 目录结构

```
bodyreader_msg/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Vector2f.msg            # 2D 向量
    ├── Vector3f.msg            # 3D 向量
    ├── Joint.msg               # 单个关节(类型 + 深度像素 + 世界坐标)
    ├── Body.msg                # 单个人体(中心 + 19 关节)
    ├── Bodylist.msg            # 多人列表(最多 6 人)
    ├── Bodyposture.msg         # 人体姿态(向上层 follower / interaction 输出)
    ├── Maskdata.msg            # 人体掩膜数据
    ├── Lockedmaskwh.msg        # 锁定目标的掩膜宽高
    └── Lockedcharrgb.msg       # 锁定目标的特征 RGB
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 主要消息定义

### `Vector2f.msg` / `Vector3f.msg`

```
float32 x
float32 y
[float32 z]
```

### `Joint.msg`

```
int8 type                # 关节编号(0~18)
Vector2f depthposition   # 深度图像素坐标
Vector3f worldposition   # 三维世界坐标(m)
```

### `Body.msg`

```
int16 bodyid             # 人体 ID
Vector3f centerofmass    # 重心
Joint[19] joints         # 19 个关键点
```

### `Bodylist.msg`

```
int8 count               # 当前帧人数(0~6)
Body[6] bodies
```

### `Bodyposture.msg`

人体姿态(具体字段以源文件为准,通常包含距离、角度、举手等状态)。

### `Maskdata.msg` / `Lockedmaskwh.msg` / `Lockedcharrgb.msg`

人体分割掩膜数据、目标锁定的尺寸/RGB 特征。

## 编译

```bash
colcon build --packages-select bodyreader_msg
source install/setup.bash
ros2 interface show bodyreader_msg/msg/Bodylist
```

## 注意事项

1. 19 个关节的具体编号定义参考奥比中光 SDK 文档。
2. `Bodylist` 固定大小 6,超出时仅保留前 6 个目标。
