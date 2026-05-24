# simple_follower_ros2

WheelTec 机器人多模态跟随包,提供视觉颜色跟随、激光跟随、ArUco 跟随、巡线(含随机分叉)等多种"跟随 + PID"实现。

## 概述

纯 Python 实现,包含若干"识别 + PID 跟随"节点:
- `visualFollower` — 基于 HSV 颜色块识别 + 距离估计的视觉跟随
- `visualTracker` — 视觉识别器(发布目标位置)
- `laserfollower` / `laserTracker` — 基于 2D 激光的目标跟随
- `ar_follow` — 基于 ArUco 的跟随
- `line_follow` / `line_follow_node` — 基于颜色线条的巡线
- `adjust_hsv` — HSV 阈值调试工具

## 目录结构

```
simple_follower_ros2/
├── package.xml
├── setup.py / setup.cfg
├── msg/
│   └── Position.msg                  # 目标角度 + 距离
├── param/
│   ├── PID_visual_param.yaml
│   └── pid_laser_param.yaml
├── parameters/
│   ├── PID_laser_param.yaml
│   ├── PID_visual_param.yaml
│   └── ar_param.yaml
├── resource/
├── simple_follower_ros2/
│   ├── visualFollower.py / visualTracker.py
│   ├── laserfollower.py / laserTracker.py
│   ├── ar_follow.py
│   ├── line_follow.py / line_follow_node.py
│   └── adjust_hsv.py
└── launch/
    ├── visual_follower.launch.py
    ├── laser_follower.launch.py
    ├── aruco_follower.launch.py
    ├── line_follower.launch.py
    ├── line_follow_random.launch.py
    └── adjust_hsv.launch.py
```

## 依赖项

- buildtool: `ament_python`
- depend: `rclpy`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`cv_bridge`、`OpenCV`、`numpy`、`aruco_msgs`(用于 ArUco 跟随)

## 消息定义

### `msg/Position.msg`

```
float32 angleX     # 目标方位角(rad,左正右负)
float32 angleY     # 目标俯仰角
float32 distance   # 估算距离(m)
```

## 节点说明

### `visualFollower`、`visualTracker`(视觉颜色跟随)

- `visualTracker` 订阅 RGB 图像,基于 HSV 阈值分割颜色块,发布 `Position`(目标位置)。
- `visualFollower` 订阅 `Position`,通过 PID 输出 `cmd_vel`。

参数(`PID_visual_param.yaml`):
- `P: [1.4, 0.4]` — 线速度 / 角速度 P 增益
- `I: [0, 0]`
- `D: [0.03, 0]`

### `laserfollower`、`laserTracker`(激光跟随)

- `laserTracker` 订阅 `/scan`,选取最近障碍物方向 / 距离作为目标。
- `laserfollower` 用 PID 跟随该目标。

参数(`pid_laser_param.yaml`):
- `P: [1.6, 0.5]`
- `D: [0.03, 0.005]`

### `ar_follow`(ArUco 跟随)

订阅 `aruco_msgs/MarkerArray` 或 `aruco_msgs/Marker`,锁定指定 ID 的 Marker 并跟随。

参数(`ar_param.yaml`):
- ArUco ID、目标距离、PID 等

### `line_follow` / `line_follow_node`(巡线)

订阅 RGB 图像,基于颜色阈值提取线条质心,输出 `cmd_vel` 巡线。`line_follow_random` 在 T 字 / Y 字分叉处随机左右选择。

### `adjust_hsv`(调参工具)

弹出 OpenCV trackbar,实时调整 HSV 阈值并显示分割结果。

## 启动文件

| 文件 | 用途 |
| --- | --- |
| `visual_follower.launch.py` | 启动底盘 + 相机 + 视觉跟随 |
| `laser_follower.launch.py` | 启动底盘 + 雷达 + 激光跟随 |
| `aruco_follower.launch.py` | 启动相机 + ArUco 节点 + 跟随 |
| `line_follower.launch.py` | 启动底盘 + 相机 + 巡线 |
| `line_follow_random.launch.py` | 巡线 + 分叉随机选择 |
| `adjust_hsv.launch.py` | 启动 HSV 调试 |

## 编译与运行

```bash
colcon build --packages-up-to simple_follower_ros2
source install/setup.bash

# 视觉跟随(默认颜色)
ros2 launch simple_follower_ros2 visual_follower.launch.py
```

## 使用示例

```bash
# 1. 调 HSV 阈值
ros2 launch simple_follower_ros2 adjust_hsv.launch.py

# 2. 把 HSV 写入对应 .py 后,启动视觉跟随
ros2 launch simple_follower_ros2 visual_follower.launch.py

# 3. 也可以激光跟随(任何最近的腿/物体)
ros2 launch simple_follower_ros2 laser_follower.launch.py
```

## 注意事项

1. 视觉跟随依赖正确的 HSV 阈值,请先使用 `adjust_hsv` 标定。
2. 激光跟随会跟"最近障碍物",出门口或大空间需谨慎。
3. PID 增益是按 S300 默认尺寸/速度调好的,小车体或地毯地面需重新调参。
4. 多个跟随节点会同时争抢 `cmd_vel`,一次只能启动一个 launch。
5. `截图 2026-05-23 18-19-05.png` 为示例图,仅作参考。
