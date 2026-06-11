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

发布(**话题名严格按下面这样**,大写字母,位于 `/ultrasonic/` 命名空间下):
- `/ultrasonic/A` (sensor_msgs/Range) — frame `ultrasonic_A`
- `/ultrasonic/B` (sensor_msgs/Range) — frame `ultrasonic_B`
- `/ultrasonic/C` (sensor_msgs/Range) — frame `ultrasonic_C`
- `/ultrasonic/D` (sensor_msgs/Range) — frame `ultrasonic_D`
- `/ultrasonic/E` (sensor_msgs/Range) — frame `ultrasonic_E`
- `/ultrasonic/F` (sensor_msgs/Range) — frame `ultrasonic_F`,**仅在 `robot_type != s300_mini` 时发布**
- `/ultrasonic/points` (sensor_msgs/PointCloud2) — 把所有有效 Range 通过 `base_frame <- ultrasonic_X` 的 TF 投影成点云,可直接被 Nav2 `voxel_layer` / `obstacle_layer` 消费

> 注:话题里的字母是大写(`A`/`B`/…/`F`),不是 `ultrasonic_a` 这种带下划线的小写;`ultrasonic_a` 等只是各路超声波的 **TF frame** 名(带下划线 + 大写字母),不要把它当成话题名订阅。

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
# 应该能看到:
#   /ultrasonic/A
#   /ultrasonic/B
#   /ultrasonic/C
#   /ultrasonic/D
#   /ultrasonic/E
#   /ultrasonic/F          (s300_mini 没有)
#   /ultrasonic/points

ros2 topic echo /ultrasonic/A         # 查看单路 Range
ros2 topic echo /ultrasonic/points    # 查看融合点云

# 在 RViz 中:
#   - 添加 Range 显示,Topic 填 /ultrasonic/A 等(全大写)
#   - 添加 PointCloud2 显示,Topic 填 /ultrasonic/points
#   - 固定坐标系设为 base_footprint,各路 Range 会以对应 ultrasonic_X frame 显示
```

## 仪表盘可视化

`wheeltec_dashboard` "组件状态"页的超声波卡已接入本包：按 `ultrasonic_A..F`
TF 的真实安装位姿绘制**俯视波束图**（扇形长度=测距、近红/中黄/远绿、
∞=灰虚线），`/ultrasonic/*` 话题判活转换节点在线状态，上线自动读取
robot_type / 量程 / FOV / 点云开关参数，`/ultrasonic/points` 显示点数与
新鲜度。详见 `src/wheeltec_dashboard/README.md`。

## 注意事项

1. 必须先编译 `robot_interfaces` 包,否则无法识别 `Supersonic` 消息。
2. 不同机型的传感器安装位置不同,务必正确设置 `robot_type`。
3. 与 Nav2 联用时需要在 costmap 配置中加入 `range_sensor_layer` 或 `voxel_layer`。`range_sensor_layer` 的 `topics` 列表需要写完整的 `/ultrasonic/A` ~ `/ultrasonic/F`(注意大写),`voxel_layer` / `obstacle_layer` 订阅 `/ultrasonic/points`。
4. 超声波回波易受干扰,建议根据实际测试结果调整 `min_range`、`max_range`。
