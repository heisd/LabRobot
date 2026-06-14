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

## 编译与架构修复记录 (Maintenance Logs)

### 2026-06-14: x86_64 架构编译与 WSL 兼容性修复
针对 x86_64 架构（如 WSL2、普通 PC）下的编译报错进行了专项修复，主要解决硬编码架构路径、缺失依赖库及编译器兼容性问题。

1. **wheeltec_robot_kcf**:
   - **问题**: `ImageConverter` 类缺少 `has_display` 成员导致编译失败。
   - **修复**: 补全类定义，并增加对 `DISPLAY` 环境变量的检测。在无显示环境（如 WSL 纯命令行）下自动禁用 OpenCV GUI 窗口，防止运行时崩溃。
2. **wheeltec_mic_ros2**:
   - **问题**: `CMakeLists.txt` 硬编码了 `arm64` 的库路径。
   - **修复**: 修改为通过 `uname -m` 自动检测架构，x86_64 下自动链接 `lib/x64` 目录，ARM 下链接 `lib/arm64`。
3. **astra_camera**:
   - **问题**: 源码包内缺失本地 `openni2_redist` 预编译库，导致链接与安装失败。
   - **修复**: 将依赖重定向至系统安装的 `libopenni2-dev`。修改 `CMakeLists.txt` 使用 `find_library` 定位系统库，并移除/注释掉对缺失本地目录的引用。
4. **orb_slam2_ros**:
   - **问题**: Boost 序列化库在 Ubuntu 22.04 (Humble) 下存在 `library_version_type` 类型未定义错误。
   - **修复**: 在 `BoostArchiver.h` 中增加版本宏判断，针对新版 Boost 包含正确的兼容头文件。

5. **bodyreader**:
   - **问题**: 该包强依赖奥比中光（Orbbec）的闭源 Astra Body Tracking SDK，且源码包内仅含 `aarch64` 架构库，导致 x86_64 架构下无法编译。
   - **修复**: 
     - 进行了核心逻辑的**跨平台重构**：引入了基于 **Google MediaPipe** 的 Python 实现 (`mediapipe_bodyreader.py` 和 `mediapipe_body_process.py`)。
     - 重构后的 Python 节点可以订阅标准 ROS2 图像话题，并发布与原包完全一致的 `bodyreader_msg/Bodylist` 消息。
     - 修改了 `CMakeLists.txt` 使其支持架构自适应：在 ARM64 环境下仍优先编译 C++ SDK 节点，而在 x86_64 环境下自动切换为 MediaPipe 方案。
   - **状态**: 已修复。现在该包在 x86_64/WSL 环境下已能顺利编译并通过 `final.launch.py` 启动（运行需安装 `mediapipe` 库）。

## 常用文档速查

- [`docs/wheeltec/ROS2-V3.5(humble)常用指令.txt`](docs/wheeltec/ROS2-V3.5(humble)常用指令.txt) — 移动底盘常用 ROS2 命令速查
- [`docs/lebai/DEBUG_README.md`](docs/lebai/DEBUG_README.md) — VSCode 下 ROS2 节点（C++）gdb 调试指南
- [`docs/lebai/DEBUG_ABOUT_OBJECT_GRAB.md`](docs/lebai/DEBUG_ABOUT_OBJECT_GRAB.md) — 颜色识别抓取相关问题（TF 树/可视化）调试记录
- [`docs/lebai/更新记录.txt`](docs/lebai/更新记录.txt) — 模型/功能更新记录
