# wheeltec_robot_kcf

WheelTec 机器人基于 KCF(Kernelized Correlation Filter)算法的视觉单目标跟随包,可让机器人锁定相机画面中选定的目标,并保持设定距离自主跟随。

## 概述

整合 KCF 跟踪器 + FHOG 特征 + PID 控制,实现"鼠标框选目标 → 实时跟踪 → cmd_vel 跟随"。适合人/物体的近距离视觉跟随。

## 目录结构

```
wheeltec_robot_kcf/
├── CMakeLists.txt
├── package.xml
├── README.md
├── RUNTIME_PARAM_GUIDE.md     # 运行时参数指南(英文)
├── include/
├── src/
│   ├── run_tracker.cpp        # 主节点(订阅图像、跟踪、发布 cmd_vel)
│   ├── kcftracker.cpp         # KCF 跟踪算法
│   ├── fhog.cpp               # FHOG 特征
│   └── PID.cpp                # PID 控制
└── launch/
    └── wheeltec_robot_kcf.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`sensor_msgs`、`geometry_msgs`、`cv_bridge`、`image_transport`、`OpenCV`

## 节点说明

### `run_tracker_node`

订阅:
- 相机彩色图像(由 `wheeltec_camera.launch.py` 启动的相机驱动提供)

发布:
- `cmd_vel` (geometry_msgs/Twist) — 控制底盘跟随目标

GUI:
- 弹出 OpenCV 窗口,首次运行时用鼠标拖框选目标。

主要参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `targetDist_` | `0.8` | 期望保持的距离(估算值,与相机焦距/目标尺寸相关) |
| PID 增益(代码中) | — | 控制线速度 / 角速度的比例-积分-微分系数 |

详细运行时参数见 `RUNTIME_PARAM_GUIDE.md`。

## 启动文件

### `wheeltec_robot_kcf.launch.py`

依次启动:
1. 底盘(`turn_on_wheeltec_robot.launch.py`)
2. 相机(`wheeltec_camera.launch.py`)
3. `run_tracker_node`,传入 `targetDist_` 参数

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_robot_kcf
source install/setup.bash
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf.launch.py
```

## 使用示例

```bash
# 1. 启动
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf.launch.py

# 2. 弹出图像窗口后,用鼠标拖框选取要跟随的目标
#    机器人开始保持设定距离跟随目标
```

## 注意事项

1. KCF 在目标剧烈形变 / 长时间遮挡时会丢失,需重新框选。
2. `targetDist_` 取决于目标尺寸与相机焦距,需要现场调试。
3. 跟随时会直接覆盖 `cmd_vel`,请保留键盘急停。
4. 默认依赖普通 USB 相机,如使用 RGB-D 可改 launch 中相机 include。
