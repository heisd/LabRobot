# wheeltec_ultrasonic

WheelTec S300 系列机器人超声波/红外距离数据转换包,把底盘发布的 `robot_interfaces/Supersonic` 多路距离消息拆解为标准 `sensor_msgs/Range` 与 `PointCloud2`,以便 Nav2 与 RViz 直接消费。

## 概述

WheelTec 底盘把所有(最多 8 路)超声波 / 红外距离打包在 `Supersonic` 中发送。Nav2、Costmap2D 等模块只能识别标准 `Range`,因此本包负责"拆包 + 坐标变换 + 重新发布"。

## 目录结构

```
wheeltec_ultrasonic/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   └── supersonic_converter_node.cpp    # 主节点
└── launch/
    └── supersonic+converter.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`std_msgs`、`tf2`、`tf2_ros`、`geometry_msgs`、`robot_interfaces`

## 节点说明

### `supersonic_converter_node`

订阅:
- `<input_topic>` (robot_interfaces/Supersonic) — 默认 `Distance`,由 `turn_on_wheeltec_robot` 发布

发布:
- 多个 `sensor_msgs/Range` 话题,按各路传感器名称分别发布(如 `ultrasonic_a`、`ultrasonic_b` ...)
- `/ultrasonic/points` (sensor_msgs/PointCloud2) — 把所有有效距离投影为点云,可被 voxel_layer 使用

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `robot_type` | `s300_pro` | 机型,决定各路传感器在车体的安装角度/位置 |
| `input_topic` | `Distance` | 输入话题 |
| `base_frame` | `base_footprint` | 输出参考系 |
| `publish_pointcloud` | `true` | 是否发布点云 |
| `min_range` | `0.02` | 最小测距(m) |
| `max_range` | `3.0` | 最大测距(m) |
| `field_of_view` | `0.5` | Range 消息的 FOV(rad) |

## 启动文件

### `supersonic+converter.launch.py`

启动 `supersonic_converter_node`,默认参数按 S300 Pro 配置。`wheeltec_nav2.launch.py` 在启用超声波避障时会自动包含此 launch。

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_ultrasonic
source install/setup.bash
ros2 launch wheeltec_ultrasonic supersonic+converter.launch.py
```

## 使用示例

```bash
# 1. 启动底盘(发布 Distance 话题)
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 2. 启动转换节点
ros2 launch wheeltec_ultrasonic supersonic+converter.launch.py

# 3. 验证
ros2 topic list | grep ultrasonic
ros2 topic echo /ultrasonic/points
```

## 注意事项

1. 必须先编译 `robot_interfaces` 包,否则无法识别 `Supersonic` 消息。
2. 不同机型的传感器安装位置不同,务必正确设置 `robot_type`。
3. 与 Nav2 联用时需要在 costmap 配置中加入 `range_sensor_layer` 或 `voxel_layer`。
4. 超声波回波易受干扰,建议根据实际测试结果调整 `min_range`、`max_range`。
