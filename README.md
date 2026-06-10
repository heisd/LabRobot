# Mobile Manipulator System

移动机械臂系统：**Wheeltec S300** 移动底盘 + **Lebai LM3** 机械臂，基于 **ROS 2 Humble**，
统一为单一 colcon 工作空间，可实现"移动 + 视觉识别 + 机械臂抓取"的完整流程。

本仓库由原本独立的 `wheeltec_S300`（移动底盘）与 `lebai`（机械臂）两个仓库合并而来，
所有功能包现已统一放在顶层 [`src/`](src) 目录下，共用同一套编译/依赖管理。

## 目录结构

```
LabRobot/
├── src/            # 统一的 colcon 工作空间源码（移动底盘 + 机械臂功能包）
│   ├── turn_on_wheeltec_robot/   # 底盘核心（详见 docs/wheeltec.md）
│   ├── ...                       # 其余 Wheeltec 功能包
│   ├── lebai-ros-sdk-humble-dev/ # 乐白官方 SDK（详见 docs/lebai.md）
│   ├── grab_demo/                # 视觉抓取核心包
│   ├── arm_demo/ lebai_gazebo/   # 机械臂运动学示例 / Gazebo 仿真
│   └── yolo_ros/                 # 外部依赖（git submodule，底盘 YOLO 跟随 + 抓取共用）
└── docs/
    ├── wheeltec.md   # 移动底盘：功能包说明、编译、常用命令
    ├── lebai.md      # 机械臂：功能包说明、编译、常用命令
    ├── wheeltec/     # 底盘相关速查表 / 笔记
    └── lebai/        # 机械臂调试笔记、TF 树记录等
```

## 快速开始

```bash
# 克隆并拉取子模块（yolo_ros、serial_ros2）
git clone --recurse-submodules <repo_url>
cd LabRobot
# 或已克隆的仓库：
git submodule update --init --recursive

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译整个工作空间
colcon build --packages-skip-build-finished --continue-on-error
source install/setup.bash
```

> 编译大包（含 TensorRT / MoveIt 等）时若内存不足，参考
> [`docs/lebai/VirtualMemory.md`](docs/lebai/VirtualMemory.md) 增加交换分区。

## 子系统文档

| 文档 | 内容 |
| --- | --- |
| [`docs/wheeltec.md`](docs/wheeltec.md) | Wheeltec S300 移动底盘：底盘驱动、传感器、SLAM/Nav2 导航、视觉跟随、语音交互、自动回充等功能包说明与常用命令 |
| [`docs/lebai.md`](docs/lebai.md) | Lebai LM3 机械臂：官方 SDK、视觉抓取（HSV/YOLO/KCF/ArUco/VLM）、运动学示例、Gazebo 仿真等功能包说明与常用命令 |

## 外部依赖（git submodules）

| 子模块 | 路径 | 用途 |
| --- | --- | --- |
| [yolo_ros](https://github.com/mgonzs13/yolo_ros) | `src/yolo_ros` | YOLO 检测（Ultralytics），底盘 `wheeltec_yolo` 视觉跟随与机械臂 `grab_demo` YOLO 抓取共用 |
| [serial](https://github.com/jinmenglei/serial) | `src/serial_ros2` | 串口通信库（`serial` 包），`turn_on_wheeltec_robot` 依赖 |

## 常用文档速查

- [`docs/wheeltec/ROS2-V3.5(humble)常用指令.txt`](docs/wheeltec/ROS2-V3.5(humble)常用指令.txt) — 移动底盘常用 ROS2 命令速查
- [`docs/lebai/DEBUG_README.md`](docs/lebai/DEBUG_README.md) — VSCode 下 ROS2 节点（C++）gdb 调试指南
- [`docs/lebai/DEBUG_ABOUT_OBJECT_GRAB.md`](docs/lebai/DEBUG_ABOUT_OBJECT_GRAB.md) — 颜色识别抓取相关问题（TF 树/可视化）调试记录
- [`docs/lebai/更新记录.txt`](docs/lebai/更新记录.txt) — 模型/功能更新记录
