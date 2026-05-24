# robot_interfaces

WheelTec S300 机器人系列的自定义 ROS2 接口包,集中存放消息(msg)和服务(srv)定义。

## 概述

这是一个纯接口包(无可执行节点),仅用于 `rosidl_default_generators` 生成跨语言绑定。其他业务包(如 `turn_on_wheeltec_robot`、`wheeltec_ultrasonic`、`auto_recharge_ros2`)依赖本包以使用统一的消息/服务类型。

## 目录结构

```
robot_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── Supersonic.msg     # 八路超声波距离
└── srv/
    └── SetRgb.srv         # RGB 灯带控制服务
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 消息定义

### `msg/Supersonic.msg` — 超声波/红外距离

```
std_msgs/Header header
float32 distance_a
float32 distance_b
float32 distance_c
float32 distance_d
float32 distance_e
float32 distance_f
float32 distance_g
float32 distance_h
```

字段 `distance_a` ~ `distance_h` 表示 8 路传感器测得的距离(单位 m),`header.stamp` 用于时间同步,`header.frame_id` 通常为车体坐标系。

## 服务定义

### `srv/SetRgb.srv` — 设置 RGB 灯带颜色

```
# Request
bool en      # 是否开启
uint8 r      # 红色 0-255
uint8 g      # 绿色 0-255
uint8 b      # 蓝色 0-255
---
# Response
string res   # 执行结果说明
```

## 编译

```bash
colcon build --packages-select robot_interfaces
source install/setup.bash

# 校验消息可见
ros2 interface show robot_interfaces/msg/Supersonic
ros2 interface show robot_interfaces/srv/SetRgb
```

## 使用示例

C++ 中订阅 Supersonic:
```cpp
#include "robot_interfaces/msg/supersonic.hpp"
auto sub = node->create_subscription<robot_interfaces::msg::Supersonic>(
    "Distance", 10, callback);
```

Python 中调用 SetRgb 服务:
```python
from robot_interfaces.srv import SetRgb
client = node.create_client(SetRgb, 'set_rgb_color')
req = SetRgb.Request()
req.en = True
req.r, req.g, req.b = 255, 0, 0
future = client.call_async(req)
```

## 注意事项

1. 修改 msg/srv 后必须重新编译,所有依赖该包的下游包也需重新编译。
2. 编译报 `rosidl_typesupport*` 缺失时,请先 `rosdep install`。
3. 包名 `robot_interfaces` 在 ROS2 生态中较常见,部署到含其他同名包的工作区时需注意冲突。
