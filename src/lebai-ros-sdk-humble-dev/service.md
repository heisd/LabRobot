# Jetson 分支服务实现说明文档

本文档详细说明了 Jetson 分支上各个服务的具体实现文件位置和功能。

## 目录结构

```
src/
├── arm_demo/                 # 机械臂演示示例
├── grab_demo/                # 抓取演示示例
└── lebai-ros-sdk-humble-dev/ # 乐白机器人ROS2驱动SDK
```

---

## 1. 乐白机器人核心服务 (lebai-ros-sdk-humble-dev)

### 1.1 IO 服务 (io_service)

**功能**: 控制机器人的输入输出接口，包括数字IO、模拟IO和夹爪控制

**实现文件**:
- **节点入口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/io_service/io_service_node.py`
- **服务接口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/io_service/io_service_interface.py`
- **启动文件**: `src/lebai-ros-sdk-humble-dev/lebai_driver/launch/io_service.launch.py`

**提供的ROS2服务**:
- `/io_service/set_robot_do` - 设置机器人数字输出
- `/io_service/set_robot_ao` - 设置机器人模拟输出
- `/io_service/set_robot_ao_mode` - 设置机器人模拟输出模式
- `/io_service/set_robot_ai_mode` - 设置机器人模拟输入模式
- `/io_service/set_extend_do` - 设置扩展数字输出
- `/io_service/set_extend_ao` - 设置扩展模拟输出
- `/io_service/set_flange_do` - 设置法兰数字输出
- `/io_service/set_gripper_position` - 设置夹爪位置
- `/io_service/set_gripper_force` - 设置夹爪力度

**实现细节**:
- 使用 `LebaiRobot` Python SDK 与机器人通信
- 支持配置参数: `robot_ip_address`, `has_gripper`

---

### 1.2 运动服务 (motion)

**功能**: 处理机器人的运动控制，包括轨迹规划和执行

**实现文件**:
- **节点入口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/motion/motion_node.py`
- **服务接口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/motion/motion_service_interface.py`
- **轨迹处理**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/motion/tp_trajectory_handler.py`
- **动作服务器**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/motion/trajectory_action_server.py`
- **启动文件**: `src/lebai-ros-sdk-humble-dev/lebai_driver/launch/motion.launch.py`

**实现细节**:
- 使用多线程执行器 (`MultiThreadedExecutor`)
- 同时运行 `MotionServiceInterface` 和 `TrajectoryActionServer`
- 支持配置参数: `robot_ip_address`, `controller_joint_names`, `robot_description`
- 处理轨迹点位规划和执行

---

### 1.3 机器人状态服务 (robot_state)

**功能**: 发布机器人的各种状态信息

**实现文件**:
- **节点入口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/robot_state/robot_state_node.py`
- **状态接口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/robot_state/robot_state_interface.py`
- **关节状态处理器**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/robot_state/joint_state_handler.py`
- **机器人状态处理器**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/robot_state/robot_state_handler.py`
- **IO状态处理器**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/robot_state/io_state_handler.py`
- **夹爪状态处理器**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/robot_state/gripper_state_handler.py`
- **启动文件**: `src/lebai-ros-sdk-humble-dev/lebai_driver/launch/robot_state.launch.py`

**发布的主题**:
- 关节状态 (joint_states)
- 机器人状态 (robot_state)
- IO状态 (io_state)
- 夹爪状态 (gripper_state) - 仅当 has_gripper=true 时

**实现细节**:
- 支持配置参数: `robot_ip_address`, `controller_joint_names`, `gripper_joint_names`, `has_gripper`
- 使用多个状态处理器分别处理不同类型的状态信息
- 从乐白机器人SDK获取实时状态并发布到ROS2话题

---

### 1.4 系统服务 (system_service)

**功能**: 提供机器人系统级别的控制和管理

**实现文件**:
- **节点入口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/system_service/system_service_node.py`
- **服务接口**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/system_service/system_service_interface.py`
- **启动文件**: `src/lebai-ros-sdk-humble-dev/lebai_driver/launch/system_service.launch.py`

**实现细节**:
- 使用 `LebaiRobot` Python SDK
- 提供系统级别的控制服务

---

## 2. 抓取演示服务 (grab_demo)

### 2.1 抓取服务节点

**功能**: 基于视觉的物体抓取服务，使用MoveIt进行运动规划

**实现文件**:
- **服务节点**: `src/grab_demo/src/grab_service_node.cpp`

**提供的ROS2服务**:
- `/obj_grab_service` - 物体抓取服务 (grab_demo::srv::GrabObject)

**实现细节**:
- 使用 MoveIt C++ 接口进行运动规划
- 使用 TF2 获取目标物体的位姿变换
- 调用 `/io_service/set_gripper_position` 服务控制夹爪
- 运动规划组: "manipulator"
- 参考坐标系: "base_link"
- 包含预定义的观察点 "look"

**工作流程**:
1. 打开夹爪 (位置=100)
2. 移动到观察点 "look"
3. 通过TF获取目标物体在 base_link 坐标系下的位置
4. 添加Z轴补偿 (+0.02m)
5. 规划并执行到目标位置的运动
6. 闭合夹爪 (位置=0)
7. 返回观察点 "look"

---

### 2.2 其他抓取演示节点

**视觉处理节点**:
- `src/grab_demo/src/aruco_dectet.cpp` - ArUco标记检测
- `src/grab_demo/src/charuco_dectet_node.cpp` - ChArUco标记检测节点
- `src/grab_demo/src/point_cloud_node.cpp` - 点云处理节点
- `src/grab_demo/src/point_cloud_node_ai.cpp` - AI点云处理节点
- `src/grab_demo/src/hsv_range.cpp` - HSV颜色范围检测
- `src/grab_demo/src/hsv_range_claude.cpp` - HSV颜色范围检测 (优化版)

**其他功能节点**:
- `src/grab_demo/src/camera_info_node.cpp` - 相机信息发布节点
- `src/grab_demo/src/hand_eye.cpp` - 手眼标定
- `src/grab_demo/src/nav_grab.cpp` - 导航抓取
- `src/grab_demo/src/start_grab.cpp` - 开始抓取

**启动文件**:
- `src/grab_demo/launch/aruco_grab.launch.py` - ArUco标记抓取
- `src/grab_demo/launch/color_grab.launch.py` - 颜色抓取
- `src/grab_demo/launch/hand_eye.launch.py` - 手眼标定
- `src/grab_demo/launch/start_grab.launch.py` - 开始抓取

---

## 3. 机械臂演示 (arm_demo)

**功能**: 机械臂正逆运动学演示示例

**实现文件**:
- `src/arm_demo/src/fk_demo.cpp` - 正向运动学演示
- `src/arm_demo/src/ik_demo.cpp` - 逆向运动学演示

**启动文件**:
- `src/arm_demo/launch/fk_demo.launch.py` - 正向运动学演示启动
- `src/arm_demo/launch/ik_demo.launch.py` - 逆向运动学演示启动

---

## 4. 教程示例 (lebai_tutorials)

**实现文件**:
- `src/lebai-ros-sdk-humble-dev/lebai_tutorials/scripts/io_example.py` - IO控制示例
- `src/lebai-ros-sdk-humble-dev/lebai_tutorials/scripts/move_example.py` - 运动控制示例
- `src/lebai-ros-sdk-humble-dev/lebai_tutorials/scripts/joint_state_subscriber.py` - 关节状态订阅示例

---

## 5. 工具和辅助模块

**实用工具**:
- `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/urdf_helper.py` - URDF辅助工具
- `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/param_utils.py` - 参数工具

**测试文件**:
- `src/lebai-ros-sdk-humble-dev/lebai_driver/test/test_flake8.py` - 代码风格测试
- `src/lebai-ros-sdk-humble-dev/lebai_driver/test/test_pep257.py` - 文档字符串测试
- `src/lebai-ros-sdk-humble-dev/lebai_driver/test/test_copyright.py` - 版权测试

---

## 6. 启动文件总览

**机器人接口启动**:
- `src/lebai-ros-sdk-humble-dev/lebai_driver/launch/robot_interface.launch.py` - 机器人完整接口

**MoveIt配置启动**:
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_moveit_config/launch/lm3.launch.py` - LM3 MoveIt配置
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_moveit_config/launch/lm3_l1.launch.py` - LM3-L1 MoveIt配置
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_moveit_config/launch/demo.launch.py` - MoveIt演示
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_moveit_config/launch/rviz.launch.py` - RViz可视化

**机器人显示启动**:
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/display_lm3.launch.py` - LM3显示
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/display_lm3_l1.launch.py` - LM3-L1显示
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/display_lm3_with_gripper.launch.py` - LM3带夹爪显示
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/display_lm3_l1_with_gripper.launch.py` - LM3-L1带夹爪显示

**独立运行**:
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/standalone_lm3.launch.py` - LM3独立运行
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/robot_interface_lm3.launch.py` - LM3机器人接口
- `src/lebai-ros-sdk-humble-dev/lebai_lm3_support/launch/robot_interface_lm3_l1.launch.py` - LM3-L1机器人接口

---

## 7. 服务依赖关系

```
IO服务 (io_service)
  └─> 夹爪控制

运动服务 (motion)
  └─> 轨迹规划和执行

机器人状态服务 (robot_state)
  └─> 发布各种状态信息

抓取服务 (grab_service_node)
  ├─> 依赖 IO服务 (夹爪控制)
  ├─> 依赖 MoveIt (运动规划)
  └─> 依赖 TF2 (坐标变换)
```

---

## 8. 关键配置参数

所有服务节点都需要以下基本参数:
- `robot_ip_address`: 机器人IP地址
- `controller_joint_names`: 控制器关节名称列表
- `has_gripper`: 是否有夹爪 (布尔值)
- `gripper_joint_names`: 夹爪关节名称列表 (当has_gripper=true时)
- `robot_description`: 机器人URDF描述

---

## 9. 编译和运行

**编译**:
```bash
cd /home/user/lebai
colcon build
```

**运行示例**:
```bash
# 启动IO服务
ros2 launch lebai_driver io_service.launch.py

# 启动运动服务
ros2 launch lebai_driver motion.launch.py

# 启动机器人状态服务
ros2 launch lebai_driver robot_state.launch.py

# 启动抓取演示
ros2 launch grab_demo start_grab.launch.py
```

---

**文档创建时间**: 2026-01-01
**分支**: Jetson