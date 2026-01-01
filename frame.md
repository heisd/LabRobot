# LeBai ROS 2 SDK 架构文档

## 项目概述

这是一个完整的 LeBai 工业机械臂 ROS 2 SDK 项目，用于控制和演示 LeBai LM3 系列协作机械臂。项目集成了硬件驱动、运动规划和视觉应用，提供从底层控制到上层应用的完整解决方案。

## 目录结构

```
lebai/
├── src/                              # 源代码目录
│   ├── arm_demo/                     # 机械臂运动学演示
│   │   ├── src/                      # FK/IK 演示节点
│   │   └── launch/                   # 启动脚本
│   ├── grab_demo/                    # 视觉抓取应用
│   │   ├── src/                      # 10+ 个演示节点
│   │   ├── include/                  # C++ 头文件
│   │   ├── launch/                   # 启动脚本
│   │   └── srv/                      # 自定义服务定义
│   └── lebai-ros-sdk-humble-dev/     # 核心 ROS 2 SDK
│       ├── lebai_driver/             # Python 驱动程序（核心）
│       ├── lebai_interfaces/         # 消息和服务定义
│       ├── lebai_lm3_support/        # 机械臂 URDF/Mesh
│       ├── lebai_lm3_moveit_config/  # MoveIt 运动规划配置
│       ├── lebai_resources/          # 资源文件
│       └── lebai_tutorials/          # 教程和示例
├── build/                            # CMake 编译输出
├── install/                          # 安装文件
└── log/                              # 构建日志
```

## 系统架构

### 分层架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         应用层 (Application Layer)               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  arm_demo   │  │  grab_demo  │  │    用户自定义应用        │  │
│  │  FK/IK演示  │  │  视觉抓取   │  │                         │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                       规划层 (Planning Layer)                    │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                       MoveIt 2                              ││
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐ ││
│  │  │ IK求解器 │  │运动规划器│  │ 碰撞检测 │  │  轨迹执行   │ ││
│  │  │  (KDL)   │  │  (OMPL)  │  │          │  │             │ ││
│  │  └──────────┘  └──────────┘  └──────────┘  └─────────────┘ ││
│  └─────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────┤
│                       驱动层 (Driver Layer)                      │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    lebai_driver (Python)                 │   │
│  │  ┌────────────┐ ┌────────────┐ ┌───────────┐ ┌────────┐ │   │
│  │  │robot_state │ │ io_service │ │  motion   │ │ system │ │   │
│  │  │  状态发布  │ │  IO控制    │ │  运动控制 │ │ 系统   │ │   │
│  │  └────────────┘ └────────────┘ └───────────┘ └────────┘ │   │
│  └──────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                       通信层 (Communication Layer)               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                 lebai Python SDK (TCP/IP)                │   │
│  └──────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                       硬件层 (Hardware Layer)                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              LeBai LM3 机械臂 + 夹爪 + 传感器             │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 数据流架构 (Mermaid)

```mermaid
%%{init: {'theme': 'base', 'themeVariables': { 'primaryColor': '#4a9eff', 'primaryTextColor': '#fff'}}}%%

flowchart TB
    subgraph HARDWARE["🔧 硬件层"]
        ROBOT["🦾 LeBai LM3<br/>6-DOF 机械臂"]
        GRIPPER["🤏 夹爪"]
        CAM["📷 相机"]
    end

    subgraph DRIVER["lebai_driver 驱动层"]
        ROBOT_STATE["robot_state_node<br/>状态监听"]
        IO_SERVICE["io_service_node<br/>IO控制"]
        MOTION["motion_node<br/>运动控制"]
        SYSTEM["system_service_node<br/>系统控制"]
    end

    subgraph VISION["grab_demo 视觉层"]
        CHARUCO["charuco_detect<br/>棋盘检测"]
        ARUCO["aruco_detect<br/>标记检测"]
        HSV["hsv_range<br/>颜色分割"]
        HAND_EYE["hand_eye<br/>手眼标定"]
    end

    subgraph PLANNING["规划层"]
        MOVEIT["MoveIt 2<br/>运动规划"]
        TF2["TF2<br/>坐标变换"]
    end

    subgraph APPLICATION["应用层"]
        GRAB["grab_service<br/>抓取服务"]
        FK["fk_demo<br/>正向运动学"]
        IK["ik_demo<br/>逆向运动学"]
        NAV["nav_grab<br/>导航抓取"]
    end

    subgraph TOPICS["Topics"]
        T1["/camera/image_raw"]
        T2["/joint_states"]
        T3["/robot_status"]
        T4["/io_status"]
        T5["/target/pose"]
    end

    subgraph SERVICES["Services"]
        S1["/motion_service/move_joint"]
        S2["/motion_service/move_line"]
        S3["/io_service/set_gripper_*"]
        S4["/system_service/enable"]
    end

    subgraph ACTION["Action"]
        A1["/lebai_trajectory_controller<br/>FollowJointTrajectory"]
    end

    %% 硬件连接
    ROBOT <--> ROBOT_STATE
    ROBOT <--> MOTION
    GRIPPER <--> IO_SERVICE
    ROBOT <--> SYSTEM
    CAM --> T1

    %% 视觉数据流
    T1 --> CHARUCO & ARUCO & HSV
    CHARUCO --> HAND_EYE
    ARUCO --> T5
    HSV --> T5

    %% TF和位姿
    T5 --> TF2
    HAND_EYE --> TF2
    TF2 --> GRAB

    %% 状态发布
    ROBOT_STATE --> T2 & T3
    IO_SERVICE --> T4

    %% 规划和执行
    GRAB --> MOVEIT
    FK & IK --> MOVEIT
    NAV --> MOVEIT
    MOVEIT --> A1
    A1 --> MOTION

    %% 服务调用
    S1 & S2 -.-> MOTION
    S3 -.-> IO_SERVICE
    S4 -.-> SYSTEM

    %% 样式定义
    classDef hw fill:#607d8b,stroke:#455a64,color:#fff
    classDef driver fill:#4caf50,stroke:#388e3c,color:#fff
    classDef vision fill:#9c27b0,stroke:#7b1fa2,color:#fff
    classDef planning fill:#ff9800,stroke:#f57c00,color:#fff
    classDef app fill:#2196f3,stroke:#1565c0,color:#fff
    classDef topic fill:#00bcd4,stroke:#0097a7,color:#fff
    classDef service fill:#ffc107,stroke:#ffa000,color:#000
    classDef action fill:#e91e63,stroke:#c2185b,color:#fff

    class ROBOT,GRIPPER,CAM hw
    class ROBOT_STATE,IO_SERVICE,MOTION,SYSTEM driver
    class CHARUCO,ARUCO,HSV,HAND_EYE vision
    class MOVEIT,TF2 planning
    class GRAB,FK,IK,NAV app
    class T1,T2,T3,T4,T5 topic
    class S1,S2,S3,S4 service
    class A1 action
```

### 数据流架构 (ASCII)

```
┌───────────────┐        ┌─────────────────┐        ┌──────────────┐
│   相机节点    │───────>│   视觉处理节点   │───────>│   TF2 坐标   │
│  (图像发布)   │        │ (ArUco/ChArUco) │        │   变换树     │
└───────────────┘        └─────────────────┘        └──────┬───────┘
                                                          │
                         ┌─────────────────┐              │
                         │   抓取服务节点   │<─────────────┘
                         │ (grab_service)  │
                         └────────┬────────┘
                                  │
                         ┌────────▼────────┐
                         │     MoveIt      │
                         │   (运动规划)    │
                         └────────┬────────┘
                                  │
              ┌───────────────────┼───────────────────┐
              ▼                   ▼                   ▼
    ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
    │  motion_node    │ │  io_service     │ │  robot_state    │
    │  (轨迹执行)     │ │  (夹爪控制)     │ │  (状态反馈)     │
    └────────┬────────┘ └────────┬────────┘ └────────┬────────┘
             │                   │                   │
             └───────────────────┼───────────────────┘
                                 ▼
                    ┌─────────────────────────┐
                    │    LeBai 机器人硬件     │
                    └─────────────────────────┘
```

## 核心模块详解

### 1. lebai_driver（核心驱动）

位置：`src/lebai-ros-sdk-humble-dev/lebai_driver/`

提供 4 个主要 ROS 2 节点：

| 节点名称 | 功能 | 发布/服务 |
|---------|------|----------|
| robot_state | 机器人状态监听 | `/robot_status`, `/joint_states`, `/io_status` |
| io_service | IO 控制 | `/io_service/set_robot_do`, `/io_service/set_gripper_*` |
| motion | 运动控制 | `/motion_service/move_joint`, `/motion_service/move_line` |
| system_service | 系统控制 | enable/disable/power_on/power_off 等 11 个服务 |

### 2. grab_demo（视觉抓取应用）

位置：`src/grab_demo/`

| 节点 | 功能描述 |
|------|---------|
| charuco_dectet_node | ChArUco 棋盘检测与位姿估计 |
| hand_eye | 手眼标定计算 |
| aruco_dectet | ArUco 标记实时检测 |
| hsv_range | HSV 颜色分割 |
| grab_service_node | 抓取服务（TF + MoveIt + 夹爪） |
| nav_grab | 导航与抓取集成 |

### 3. arm_demo（运动学演示）

位置：`src/arm_demo/`

- **fk_demo**: 正向运动学演示 - 根据关节角度计算末端位姿
- **ik_demo**: 逆向运动学演示 - 根据末端位姿计算关节角度

### 4. lebai_interfaces（接口定义）

位置：`src/lebai-ros-sdk-humble-dev/lebai_interfaces/`

**消息类型 (14 个)**：
- `RobotStatus.msg` - 机器人整体状态
- `IOStatus.msg` - IO 状态
- `GripperStatus.msg` - 夹爪状态
- `TrajectoryMoveJoint.msg` - 关节轨迹
- `TrajectoryMoveLine.msg` - 直线轨迹
- ...

**服务类型 (7 个)**：
- `MoveJoint.srv` - 关节空间运动
- `MoveLine.srv` - 笛卡尔直线运动
- `SetDO.srv` / `SetAO.srv` - IO 控制
- `SetGripper.srv` - 夹爪控制

### 5. lebai_lm3_moveit_config（MoveIt 配置）

位置：`src/lebai-ros-sdk-humble-dev/lebai_lm3_moveit_config/`

- **kinematics.yaml** - KDL 运动学插件配置
- **ompl_planning.yaml** - OMPL 路径规划算法配置
- **joint_limits.yaml** - 关节限制参数
- **ros_controllers.yaml** - 控制器配置
- **SRDF 文件** - 碰撞检测规则

## 技术栈

| 技术 | 版本/说明 | 用途 |
|------|----------|------|
| ROS 2 Humble | LTS | 机器人操作系统框架 |
| Python 3 | 3.10+ | 驱动程序实现 |
| C++ | 11/17 | 应用层开发 |
| OpenCV | 4.10 | 计算机视觉处理 |
| Eigen3 | - | 矩阵与坐标变换 |
| MoveIt 2 | - | 运动规划框架 |
| TF2 | - | 坐标变换管理 |
| URDF/Xacro | - | 机器人模型描述 |

## 快速启动

### 1. 启动核心驱动

```bash
# 启动机器人接口（包含所有驱动节点）
ros2 launch lebai_driver robot_interface.launch.py robot_ip:=<机器人IP>
```

### 2. 启动 MoveIt

```bash
ros2 launch lebai_lm3_moveit_config demo.launch.py
```

### 3. 运行演示

```bash
# 正向运动学演示
ros2 launch arm_demo fk_demo.launch.py

# 逆向运动学演示
ros2 launch arm_demo ik_demo.launch.py

# 抓取演示
ros2 launch grab_demo start_grab.launch.py

# 手眼标定
ros2 launch grab_demo hand_eye.launch.py
```

## ROS 2 话题与服务

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/robot_status` | `lebai_interfaces/RobotStatus` | 机器人状态 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/io_status` | `lebai_interfaces/IOStatus` | IO 状态 |

### 提供的服务

| 服务名称 | 类型 | 描述 |
|---------|------|------|
| `/motion_service/move_joint` | `MoveJoint` | 关节空间运动 |
| `/motion_service/move_line` | `MoveLine` | 笛卡尔直线运动 |
| `/io_service/set_robot_do` | `SetDO` | 设置数字输出 |
| `/io_service/set_gripper_position` | `SetGripper` | 夹爪位置控制 |
| `/system_service/enable` | `Trigger` | 使能机器人 |
| `/system_service/disable` | `Trigger` | 去使能机器人 |

### 动作服务

| 动作名称 | 类型 | 描述 |
|---------|------|------|
| `/lebai_trajectory_controller` | `FollowJointTrajectory` | 轨迹跟踪执行 |

## 依赖关系

```
grab_demo
├── rclcpp
├── sensor_msgs
├── geometry_msgs
├── OpenCV 4.10
├── Eigen3
├── tf2
├── MoveIt 2
└── lebai_interfaces

arm_demo
├── rclcpp
├── moveit_ros_planning_interface
└── moveit_core

lebai_driver
├── rclpy
├── lebai (Python SDK)
├── std_msgs
├── control_msgs
├── tf_transformations
└── lebai_interfaces
```

## 硬件支持

### 支持的机械臂型号

- **LeBai LM3** - 6 自由度协作机械臂
- **LeBai LM3-L1** - 长臂版本
- 支持双臂配置

### 夹爪支持

- 多关节夹爪（6 个关节）
- 支持位置控制和力控制

## 开发指南

### 添加新的演示应用

1. 在 `src/` 下创建新的 ROS 2 包
2. 依赖 `lebai_interfaces` 使用标准接口
3. 通过服务调用控制机器人

### 扩展驱动功能

1. 在 `lebai_driver/` 中添加新的服务处理器
2. 在 `lebai_interfaces/` 中定义新的消息/服务类型
3. 更新 launch 文件

## 相关资源

- [LeBai ROS SDK 官方文档](https://lebai-robotics.github.io/lebai-ros-sdk/)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [MoveIt 2 文档](https://moveit.ros.org/)