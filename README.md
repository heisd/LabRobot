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
git clone --recurse-submodules --branch origin_code https://github.com/heisd/LabRobot.git
cd LabRobot
# 或已克隆的仓库：
git submodule update --init --recursive

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译整个工作空间
source /opt/ros/humble/setup.bash
colcon build --symlink-install
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

- [`docs/wheeltec/ROS2-V3.5(humble)常用指令.txt`](docs/wheeltec/ROS2-V3.5%28humble%29常用指令.txt) — 移动底盘常用 ROS2 命令速查
- [`docs/lebai/DEBUG_README.md`](docs/lebai/DEBUG_README.md) — VSCode 下 ROS2 节点（C++）gdb 调试指南
- [`docs/lebai/DEBUG_ABOUT_OBJECT_GRAB.md`](docs/lebai/DEBUG_ABOUT_OBJECT_GRAB.md) — 颜色识别抓取相关问题（TF 树/可视化）调试记录
- [`docs/lebai/更新记录.txt`](docs/lebai/更新记录.txt) — 模型/功能更新记录

## 环境与系统分层

主工作空间以 ROS 2 Humble 为基础，底盘、机械臂、视觉与交互包共用一份 `src/`，但各子系统的硬件和外部依赖仍需分别配置。

| 层次 | 主要内容 | 首要验证 |
| --- | --- | --- |
| 底盘 | 串口驱动、轮速、里程计与传感器 | 通信、TF、里程计 |
| 导航 | SLAM、定位、Nav2 与巡航 | 地图、坐标系、目标可达性 |
| 机械臂 | SDK、状态、MoveIt 与轨迹 | 连接、关节与控制器 |
| 视觉抓取 | 相机、识别、TF 与抓取服务 | 相机内参、手眼标定、目标坐标 |
| 交互 | Web、视频、语音等 | 后端接口与依赖服务 |

移动机械臂的完整动作依赖多层协同；仓库包含这些功能包，不代表首次克隆后无需标定即可执行所有流程。

## 首次构建与子模块检查

```bash
git submodule status
colcon list
```

克隆时若没有带 `--recurse-submodules`，先在仓库根目录执行前文的子模块初始化。YOLO 和串口库的来源由 [.gitmodules](./.gitmodules) 指定，不能用一个 Python 包安装步骤替代源码子模块。

首次使用建议只构建准备验证的功能及依赖，例如：

```bash
colcon build --symlink-install --packages-up-to grab_demo
source install/setup.bash
```

该命令仍需该包依赖已安装。TensorRT、相机 SDK、机械臂 SDK、模型权重和设备参数应按子系统指南补齐。构建失败时先看首个失败包，不将后续依赖中止都视为独立故障。

## 建议验证路线

1. 按 [底盘指南](./docs/wheeltec.md) 确认车型、设备与底盘通信。
2. 按 [机械臂指南](./docs/lebai.md) 确认驱动、关节状态与模型。
3. 选择实机或 [机械臂仿真](./src/lebai_gazebo/GAZEBO_GUIDE.md)，分别核对控制器。
4. 检查图像、深度、相机内参和 TF 后，再运行识别。
5. 确认目标位姿后连接抓取服务，最后验证移动与抓取协同。

不要同时让多个未协调的控制节点向同一个执行器下发命令。底盘已有速度仲裁相关设计，详细优先级见底盘文档。

## 接口与排障入口

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

| 现象 | 建议排查 |
| --- | --- |
| 找不到 serial / YOLO 消息包 | 子模块状态、依赖与重新构建 |
| 相机画面正常但抓取偏移 | 深度、内参、手眼标定与 TF |
| MoveIt 可规划但实体不执行 | SDK 连接、控制器名称和 Action 服务 |
| 底盘移动指令没有效果 | 当前控制来源、仲裁话题与下位机通信 |
| 仿真与实机行为不一致 | 使用的 launch、硬件接口与控制器配置 |
| 大包编译资源不足 | 降低并行度、缩小包范围，再参考虚拟内存笔记 |

[docs/](./docs) 和各包 README 是详细参数的主要来源；根目录仅汇总整体关系与首次操作路线。本次文档整理没有替代实际硬件、标定或全工作空间运行验证。
