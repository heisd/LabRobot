# auto_recharge_ros2

WheelTec S300 机器人 ROS2 自动回充包,基于电压阈值与预先记录的充电桩位姿,自动调用 Nav2 完成"低电量返回充电桩 + 红外对接"流程。

## 概述

包内 Python 节点会:
1. 监听电池电压、充电状态、红外避障状态。
2. 当电压低于阈值时,把预先保存到 `Charger_Position.json` 的充电桩位姿作为 Nav2 目标点发布,并在 RViz 中以 Marker 显示。
3. 接近充电桩后切换为红外对接小步速度,通过 `cmd_vel` 微调直至检测到 `robot_charging_flag`。
4. 充电完成或电压恢复后自动恢复正常作业。

## 目录结构

```
auto_recharge_ros2/
├── package.xml
├── setup.py / setup.cfg
├── Charger_Position.json          # 充电桩位姿(可被运行时更新)
├── robot_info.yaml                # 机器人电池/类型参数
├── resource/
└── auto_recharge_ros2/
    ├── __init__.py
    └── auto_recharger.py          # 主节点
```

## 依赖项

- buildtool: `ament_python`
- depend: `rclpy`、`geometry_msgs`、`nav_msgs`、`std_msgs`、`visualization_msgs`、`nav2_msgs`、`tf2_ros`

## 节点说明

### `auto_recharger`(可执行文件:`auto_recharge`)

订阅:
- `PowerVoltage` (std_msgs/Float32) — 电池电压
- `robot_charging_flag` (std_msgs/Bool) — 是否在充电中
- `robot_charging_current` (std_msgs/Float32) — 充电电流
- `robot_red_flag` (std_msgs/Bool) — 红外避障状态
- `/charger_position_update` (geometry_msgs/PoseStamped) — 运行时更新充电桩位姿
- `/odom` (nav_msgs/Odometry) — 里程计

发布:
- `/chassis_security` (std_msgs/Int8) — 通知底盘进入安全停车
- `/goal_marker` (visualization_msgs/MarkerArray) — 充电桩可视化标记
- `robot_recharge_flag` (std_msgs/Int8) — 通知底盘启用回充流程
- `/cmd_vel` (geometry_msgs/Twist) — 红外对接阶段直接控制速度

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `robot_BatteryCapacity` | `5000` | 电池容量(mAh)|
| `car_mode` | `mini_mec` | 车型 |
| `diff_point` | `1.2` | 距离充电桩多远时切换红外对接 |
| `diff_angle` | `-15` | 对接角度补偿 |

## 配置文件

### `Charger_Position.json`

```json
{
  "p_x": 0.0, "p_y": 0.0, "p_z": 0.0,
  "orien_x": 0.0, "orien_y": 0.0,
  "orien_z": 0.0, "orien_w": 1.0
}
```

由用户首次手动停泊充电桩后保存,运行时也可通过 `/charger_position_update` 话题更新。

### `robot_info.yaml`

存放机器人电池容量、车型等参数,启动时被 launch 文件读取并注入到节点参数。

## 编译与运行

```bash
colcon build --packages-select auto_recharge_ros2
source install/setup.bash
ros2 run auto_recharge_ros2 auto_recharge
```

> 注:`setup.py` 中的 `console_scripts` 入口为
> `auto_recharge = auto_recharge_ros2.auto_recharger:main`,因此
> 命令行可执行名为 **`auto_recharge`**(而不是模块文件名
> `auto_recharger`)。

## 使用示例

```bash
# 1. 启动底盘 + 雷达 + Nav2(包含定位)
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

# 2. 手动将机器人推到充电桩前,记录当前位姿到 Charger_Position.json
# (或运行时通过 /charger_position_update 发布)

# 3. 启动自动回充
ros2 run auto_recharge_ros2 auto_recharge

# 电压低于阈值后,节点会自动:
# - 调用 Nav2 导航到充电桩附近
# - 切换为红外对接精对正
# - 检测到充电后停止
```

## 注意事项

1. 必须先有可用的 Nav2 定位与地图。
2. `Charger_Position.json` 路径在源码中硬编码为 `/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json`,部署到其他位置时需要修改源码或软链接。
3. 红外对接阶段会直接控制 `cmd_vel`,会覆盖其他遥控命令,请确保此时无其他来源。
4. `robot_BatteryCapacity` 必须与实际电池容量一致,否则电压阈值可能不准。
5. 充电桩位姿改变(地图更新、桩移动)后必须重新保存。
