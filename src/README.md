# lebai 工作空间（src）

本目录是 **乐白（Lebai）LM3 机械臂** 的 ROS 2（Humble）工作空间源码目录，集成了官方 SDK、
机械臂运动学示例、多种视觉抓取方案以及 Gazebo 仿真场景。配合移动底盘（wheeltec），可实现
"移动 + 视觉识别 + 机械臂抓取" 的完整流程。

> 文档约定：本目录及各子包内的 `*.md` 指南均为中文，本 README 为整个 `src/` 的总览索引。

## 目录结构

| 子目录 / 文件 | 说明 |
|--------------|------|
| `lebai-ros-sdk-humble-dev/` | 乐白官方 ROS 2 SDK：驱动、接口、URDF 模型、MoveIt 配置、示例教程 |
| `grab_demo/` | 视觉抓取核心包：HSV / YOLO / KCF / ArUco / VLM 多种识别方案与抓取服务 |
| `arm_demo/` | 机械臂运动学示例：正运动学（FK）/ 逆运动学（IK）演示 |
| `lebai_gazebo/` | LM3 机械臂的 Gazebo Classic 仿真场景与启动文件 |
| `DEBUG_README.md` | VSCode 下 ROS 2 节点（C++）的 gdb 调试指南 |
| `DEBUG_ABOUT_OBJECT_GRAB.md` | 颜色识别抓取相关问题（TF 树 / 可视化）的调试记录 |
| `VirtualMemory.md` | 编译时增加交换分区（虚拟内存）的方法，避免大包编译 OOM |
| `wheeltec_S300常用指令.txt` | wheeltec S300 移动底盘的常用命令速查 |
| `更新记录.txt` | 模型 / 功能更新记录 |

## 子包概览

### lebai-ros-sdk-humble-dev — 官方 SDK

乐白 LM3 机械臂的官方 ROS 2 SDK，包含以下子包：

| 子包 | 作用 |
|------|------|
| `lebai_driver` | 机械臂驱动 |
| `lebai_interfaces` | 自定义消息 / 服务接口 |
| `lebai_resources` | 公共材质 / 颜色 / 常量等 xacro 资源 |
| `lebai_lm3_support` | LM3 机械臂及夹爪的 URDF / xacro 模型 |
| `lebai_lm3_moveit_config` | MoveIt 运动规划配置（含 RViz、launch） |
| `lebai_tutorials` | C++ / Python 运动与 IO 示例 |

> 安装与使用文档：<https://lebai-robotics.github.io/lebai-ros-sdk/index.html>

### grab_demo — 视觉抓取

抓取功能的核心包，提供多种视觉识别方案，它们**共用同一套相机接口与抓取流程**
（统一通过 `target_frame` 的 TF 广播 → `grab_service_node` 执行抓取），可按需切换：

| 方案 | 节点 | 说明 | 指南 |
|------|------|------|------|
| HSV 颜色 | `hsv_range` | 按 HSV 阈值找最大色块，最轻量 | `HSV_GUIDE.md` |
| YOLO 检测 (TensorRT) | `yolo_detect_node` | 自研 TensorRT 版 YOLOv8（需 CUDA+TensorRT，仅 Jetson） | `YOLO_GUIDE.md` |
| YOLO 检测 (yolo_ros) | `yolo_ros_detect_node.py` | 官方 [yolo_ros](https://github.com/mgonzs13/yolo_ros)（ultralytics）+ 闭环抓取，CPU/GPU 皆可 | `YOLO_ROS_GUIDE.md` |
| KCF 跟踪 | `kcf_track_node` | KCF 相关滤波跟踪，需初始框（可由 HSV 自动播种） | `KCF_GUIDE.md` |
| ArUco | `aruco_dectet` | ArUco 标记识别 | — |
| VLM 自然语言 | `vlm_grab_node.py` | 视觉语言模型，按一句自然语言选物抓取 | `VLM_GUIDE.md` |

其它关键节点 / 资源：

- `grab_service_node` — 抓取服务（订阅 `target_frame`，驱动机械臂抓取，开环）
- `closed_loop_grab_node` — 闭环（PBVS）抓取服务：看-动-再看-修正后再抓，接口与开环版一致；并接入抓取仲裁可被手动打断（见 `YOLO_ROS_GUIDE.md`）
- `arm_arbiter_node.py` — 抓取仲裁：手动优先，可随时打断 YOLO/KCF/HSV/VLM 的自动抓取（见 `ARBITER_GUIDE.md`）
- `start_grab` — 抓取流程入口
- `hand_eye` / `charuco_dectet_node` — 手眼标定及 ChArUco 标定板识别（见 `point_cloud.md`、根目录 `serivce.md`）
- `camera_info_node` / `point_cloud_node` — 相机内参与点云处理
- `nav_grab` — 导航 + 抓取（移动底盘联动）
- `srv/GrabObject.srv` — 抓取服务接口
- `launch/` — 各方案的启动文件：`color_grab` / `yolo_grab` / `yolo_ros_grab`（yolo_ros + 闭环）/ `kcf_grab` / `aruco_grab` / `vlm_grab` / `hand_eye` / `start_grab`
- 调试参见 `DEBUG_GUIDE.md`

### arm_demo — 运动学示例

机械臂运动学最小示例：

- `fk_demo` — 正运动学演示（`launch/fk_demo.launch.py`）
- `ik_demo` — 逆运动学演示（`launch/ik_demo.launch.py`）

### lebai_gazebo — Gazebo 仿真

LM3 机械臂的 Gazebo Classic（11）仿真场景：地面 + 桌子 + 标准物体 + 机械臂，可在仿真中
运行抓取 / 视觉 / VLM 等功能。详见 `lebai_gazebo/GAZEBO_GUIDE.md`。

```bash
ros2 launch lebai_gazebo gazebo.launch.py        # 仿真场景
ros2 launch lebai_gazebo gazebo_grab.launch.py   # 仿真 + 抓取
```

## 环境要求

- **ROS 2 Humble**（Ubuntu 22.04）
- **OpenCV / cv_bridge**（视觉处理）
- YOLO（TensorRT 版）需 **TensorRT**；YOLO（yolo_ros 版）需 **ultralytics**（`pip install ultralytics`，CPU/GPU 皆可，见 `grab_demo/YOLO_ROS_GUIDE.md`）；VLM 方案需可访问的视觉语言模型（OpenAI 兼容 / Anthropic）
- yolo_ros 以 **git submodule** 形式置于 `src/yolo_ros`，克隆后需 `git submodule update --init --recursive`
- 仿真需 **Gazebo Classic 11** 及 `gazebo_ros2_control` 等插件（见 `GAZEBO_GUIDE.md`）

> 编译大包（如含 TensorRT / MoveIt）时若内存不足，请参考 `VirtualMemory.md` 增加交换分区。

## 编译

本目录即 colcon 工作空间的 `src/`，在其上一级（工作空间根目录）执行：

```bash
cd ~/lebai
colcon build              # 编译全部
source install/setup.bash

# 或只编译某个包
colcon build --packages-select grab_demo
```

## 快速开始

以颜色（HSV）抓取为例：

```bash
source install/setup.bash
ros2 launch grab_demo color_grab.launch.py
```

切换其它视觉方案时，改用对应 launch（`yolo_grab` / `kcf_grab` / `aruco_grab` / `vlm_grab`）即可，
抓取服务侧无需改动。各方案的参数、话题与注意事项见 `grab_demo/` 下对应的 `*_GUIDE.md`。

## 相关文档

- 视觉抓取调试：`grab_demo/DEBUG_GUIDE.md`、`DEBUG_ABOUT_OBJECT_GRAB.md`
- C++ 节点 gdb 调试：`DEBUG_README.md`
- 手眼标定 / 点云：`grab_demo/point_cloud.md`、根目录 `serivce.md`、`frame.md`
- 仿真：`lebai_gazebo/GAZEBO_GUIDE.md`
