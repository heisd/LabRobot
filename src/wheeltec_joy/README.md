# wheeltec_joy

WheelTec S300 系列机器人的 ROS2 手柄控制包,将 `/joy` 话题中的轴/按键数据映射为 `cmd_vel` 速度指令,实现手柄遥控。

## 概述

该包封装了一个简单的 joy-to-Twist 转换节点,通过 ROS2 `joy` 驱动节点读取手柄,再发布机器人速度指令。

## 目录结构

```
wheeltec_joy/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── wheeltec_joy.launch.py    # 启动 joy 节点 + 控制节点
│   └── only_joy.launch.py        # 只启动控制节点(适用于已经存在 joy 节点的场景)
└── src/
    └── wheeltec_joy_control.cpp  # 主节点
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`geometry_msgs`、`sensor_msgs`、`joy`

## 节点说明

### `wheeltec_joy`(可执行文件:`wheeltec_joy_node`)

订阅:
- `/joy` (sensor_msgs/Joy) — 手柄输入

发布:
- `cmd_vel` (geometry_msgs/Twist) — 速度指令

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `axis_linear` | `1` | 用于线速度的手柄轴序号(默认 `axes[1]`) |
| `axis_angular` | `0` | 用于角速度的手柄轴序号(默认 `axes[0]`) |
| `v_linear` | `0.3` | 最大线速度,单位 m/s |
| `v_angular` | `1.0` | 最大角速度,单位 rad/s |

## 启动文件

- **`wheeltec_joy.launch.py`** — 同时启动 `joy/joy_node` 与 `wheeltec_joy_node`,适合裸机器人直接遥控。
- **`only_joy.launch.py`** — 仅启动 `wheeltec_joy_node`,当系统中已经存在 joy 节点时使用。

## 编译与运行

```bash
colcon build --packages-select wheeltec_joy
source install/setup.bash
ros2 launch wheeltec_joy wheeltec_joy.launch.py
```

## 使用示例

```bash
# 1. 插上手柄(/dev/input/jsX 出现)
# 2. 启动 turn_on_wheeltec_robot 提供 cmd_vel 接收端
# 3. 启动手柄控制
ros2 launch wheeltec_joy wheeltec_joy.launch.py

# 推手柄左摇杆即可控制机器人前进/转向
```

## 注意事项

1. 默认手柄设备 `/dev/input/js0`,使用其他设备需要给 `joy_node` 设置 `device_id` 参数。
2. 不同手柄(Xbox/PS4/北通)的轴序号不同,可通过 `ros2 topic echo /joy` 调整 `axis_linear` 与 `axis_angular`。
3. 当 `wheeltec_joy_node` 与 `wheeltec_robot_keyboard` 等其他遥控同时运行时会争抢 `cmd_vel`,建议二选一。
