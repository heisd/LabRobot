# ROS2 服务接口定义与实现映射文档

本文档详细说明了项目中所有 ROS2 服务（.srv 文件）的定义以及它们在源代码中的具体实现位置和实现逻辑。

---

## 目录

1. [IO 服务 (lebai_interfaces)](#1-io-服务-lebai_interfaces)
2. [运动服务 (lebai_interfaces)](#2-运动服务-lebai_interfaces)
3. [系统服务 (std_srvs)](#3-系统服务-std_srvs)
4. [抓取服务 (grab_demo)](#4-抓取服务-grab_demo)

---

## 1. IO 服务 (lebai_interfaces)

### 1.1 SetDO - 设置数字输出

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/SetDO.srv`

```
uint16 pin      # 引脚号
bool value      # 输出值
---
bool code       # 返回状态码
```

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/io_service/io_service_interface.py`

**服务注册**:
- `/io_service/set_robot_do` (第24行)
- `/io_service/set_extend_do` (第30行)
- `/io_service/set_flange_do` (第33行)

**实现代码**:

```python
# 机器人数字输出 (第37-40行)
def cmd_set_robot_do(self, request: SetDO.Request, response: SetDO.Response):
    self.lebai_robot_.set_do(request.pin, request.value)
    response.code = True
    return response

# 扩展数字输出 (第47-50行)
def cmd_set_extend_do(self, request: SetDO.Request, response: SetDO.Response):
    self.lebai_robot_.set_extra_do(request.pin, request.value)
    response.code = True
    return response

# 法兰数字输出 (第67-70行)
def cmd_set_flange_do(self, request: SetDO.Request, response: SetDO.Response):
    self.lebai_robot_.set_flange_do(request.pin, request.value)
    response.code = True
    return response
```

**功能说明**:
- 设置机器人、扩展模块或法兰接口的数字输出引脚状态
- 使用 LebaiRobot SDK 的对应方法控制硬件

---

### 1.2 SetAO - 设置模拟输出

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/SetAO.srv`

```
uint16 pin      # 引脚号
float64 value   # 输出值
---
bool code       # 返回状态码
```

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/io_service/io_service_interface.py`

**服务注册**:
- `/io_service/set_robot_ao` (第25行)
- `/io_service/set_extend_ao` (第31行)

**实现代码**:

```python
# 机器人模拟输出 (第42-45行)
def cmd_set_robot_ao(self, request: SetAO.Request, response: SetAO.Response):
    self.lebai_robot_.set_ao(request.pin, request.value)
    response.code = True
    return response

# 扩展模拟输出 (第52-55行)
def cmd_set_extend_ao(self, request: SetAO.Request, response: SetAO.Response):
    self.lebai_robot_.set_extra_ao(request.pin, request.value)
    response.code = True
    return response
```

**功能说明**:
- 设置机器人或扩展模块的模拟输出引脚电压值
- 支持连续的电压值输出

---

### 1.3 SetAMode - 设置模拟IO模式

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/SetAMode.srv`

```
uint16 pin      # 引脚号
uint8 mode      # 模式
---
bool code       # 返回状态码
```

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/io_service/io_service_interface.py`

**服务注册**:
- `/io_service/set_robot_ao_mode` (第27行)
- `/io_service/set_robot_ai_mode` (第28行)

**实现代码**:

```python
# 设置模拟输出模式 (第57-60行)
def cmd_set_robot_ao_mode(self, request: SetAMode.Request, response: SetAMode.Response):
    self.lebai_robot_.set_ao_mode(request.pin, request.mode)
    response.code = True
    return response

# 设置模拟输入模式 (第62-65行)
def cmd_set_robot_ai_mode(self, request: SetAMode.Request, response: SetAMode.Response):
    self.lebai_robot_.set_ai_mode(request.pin, request.mode)
    request.code = True  # 注意：这里原代码有bug，应该是response.code
    return request
```

**功能说明**:
- 配置模拟IO引脚的工作模式
- 支持输入/输出模式切换

---

### 1.4 SetGripper - 设置夹爪

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/SetGripper.srv`

```
float64 val     # 值（位置或力度）
---
bool ret        # 返回状态
```

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/io_service/io_service_interface.py`

**服务注册**:
- `/io_service/set_gripper_position` (第34行)
- `/io_service/set_gripper_force` (第35行)

**实现代码**:

```python
# 设置夹爪位置 (第72-75行)
def cmd_set_gripper_position(self, request: SetGripper.Request, response: SetGripper.Response):
    self.lebai_robot_.set_claw(amplitude = request.val)
    response.ret = True
    return response

# 设置夹爪力度 (第77-80行)
def cmd_set_gripper_force(self, request: SetGripper.Request, response: SetGripper.Response):
    self.lebai_robot_.set_claw(force = request.val)
    response.ret = True
    return response
```

**功能说明**:
- `set_gripper_position`: 设置夹爪开合位置（0-100，0为闭合，100为张开）
- `set_gripper_force`: 设置夹爪抓取力度

**使用示例**:
```cpp
// 在 grab_service_node.cpp 中的使用
p_request->val=100;  // 张开夹爪
auto result_=lebai_pclient->async_send_request(p_request);

p_request->val=0;    // 闭合夹爪
result_=lebai_pclient->async_send_request(p_request);
```

---

## 2. 运动服务 (lebai_interfaces)

### 2.1 MoveJoint - 关节运动

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/MoveJoint.srv`

```
bool is_joint_pose              # 是否使用关节空间坐标
float64[] joint_pose            # 关节角度数组
geometry_msgs/Pose cartesian_pose  # 笛卡尔空间位姿
lebai_interfaces/MoveCommon common # 运动参数（加速度、速度、时间、半径）
---
bool ret                        # 返回状态
```

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/motion/tp_trajectory_handler.py`

**服务注册**: `/motion_service/move_joint` (第16行)

**实现代码**:

```python
# 第20-55行
def cmd_move_joint(self, request: MoveJoint.Request, response: MoveJoint.Response):
    pose_is_joint_angle = request.is_joint_pose
    acc = request.common.acc
    vel = request.common.vel
    time = request.common.time
    radius = request.common.radius

    if pose_is_joint_angle:
        # 关节空间运动
        try:
            self.lebai_robot_.movej(JointPose(request.joint_pose), acc, vel, time, radius)
            request.ret = True
        except Exception as e:
            request.ret = False
            self.node_.get_logger().error(f"Error in move_joint: {e}")
    else:
        # 笛卡尔空间运动
        quat_msg = request.cartesian_pose.orientation
        quat_tf = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        euler = tf_transformations.euler_from_quaternion(quat_tf)
        pose = CartesianPose(
            request.cartesian_pose.position.x,
            request.cartesian_pose.position.y,
            request.cartesian_pose.position.z,
            euler[2], euler[1], euler[0]
        )
        try:
            self.lebai_robot_.movej(pose, acc, vel, time, radius)
        except Exception as e:
            request.ret = False
            self.node_.get_logger().error(f"Error in move_joint: {e}")
    return response
```

**功能说明**:
- 执行关节空间（Joint Space）运动，类似于人体手臂关节转动
- 支持两种输入方式：关节角度或笛卡尔坐标
- 包含异常处理和日志记录

---

### 2.2 MoveLine - 直线运动

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/MoveLine.srv`

```
bool is_joint_pose              # 是否使用关节空间坐标
float64[] joint_pose            # 关节角度数组
geometry_msgs/Pose cartesian_pose  # 笛卡尔空间位姿
lebai_interfaces/MoveCommon common # 运动参数（加速度、速度、时间、半径）
---
bool ret                        # 返回状态
```

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/motion/tp_trajectory_handler.py`

**服务注册**: `/motion_service/move_line` (第17行)

**实现代码**:

```python
# 第57-83行
def cmd_move_line(self, request: MoveLine.Request, response: MoveLine.Response):
    pose_is_joint_angle = request.is_joint_pose
    acc = request.common.acc
    vel = request.common.vel
    time = request.common.time
    radius = request.common.radius

    if pose_is_joint_angle:
        try:
            self.lebai_robot_.movel(JointPose(request.joint_pose), acc, vel, time, radius)
        except Exception as e:
            request.ret = False
            self.node_.get_logger().error(f"Error in move_line: {e}")
    else:
        quat_msg = request.cartesian_pose.orientation
        quat_tf = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        euler = tf_transformations.euler_from_quaternion(quat_tf)
        pose = CartesianPose(
            request.cartesian_pose.position.x,
            request.cartesian_pose.position.y,
            request.cartesian_pose.position.z,
            euler[2], euler[1], euler[0]
        )
        try:
            self.lebai_robot_.movel(pose, acc, vel, time, radius)
        except Exception as e:
            request.ret = False
            self.node_.get_logger().error(f"Error in move_line: {e}")
    response.ret = True
    return response
```

**功能说明**:
- 执行直线运动，末端执行器沿直线路径移动
- 支持关节角度或笛卡尔坐标输入
- 包含异常处理机制

---

### 2.3 MoveCircle - 圆弧运动

**服务定义文件**: `src/lebai-ros-sdk-humble-dev/lebai_interfaces/srv/MoveCircle.srv`

```
bool way_point_is_joint_pose           # 途径点是否为关节坐标
float64[] way_point_joint_pose         # 途径点关节角度
geometry_msgs/Pose way_point_cartesian_pose  # 途径点笛卡尔位姿

bool end_point_is_joint_pose           # 终点是否为关节坐标
float64[] end_point_joint_pose         # 终点关节角度
geometry_msgs/Pose end_point_cartesian_pose  # 终点笛卡尔位姿

float64 circle_angle                   # 圆弧角度
lebai_interfaces/MoveCommon common     # 运动参数
---
bool ret                               # 返回状态
```

**实现位置**: 目前被注释掉，未实现

**服务注册**: 未注册（第18行被注释）

**功能说明**:
- 预留的圆弧运动服务接口
- 需要指定途径点和终点来定义圆弧路径
- 当前版本暂未启用

---

## 3. 系统服务 (std_srvs)

所有系统服务都使用标准的 `std_srvs/Empty` 服务类型（无请求参数，无响应参数）

**实现位置**: `src/lebai-ros-sdk-humble-dev/lebai_driver/lebai_driver/system_service/system_service_interface.py`

### 3.1 急停服务

**服务名**: `/system_service/emergency_stop`
**注册位置**: 第19行
**实现代码**:
```python
# 第31-33行
def cmd_emergency_stop(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.estop()
    return response
```
**功能**: 紧急停止机器人运动

---

### 3.2 电源控制服务

**服务名**:
- `/system_service/power_on` (第20行)
- `/system_service/power_off` (第21行)

**实现代码**:
```python
# 上电 (第35-37行)
def cmd_power_on(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.start_sys()
    return response

# 断电 (第39-41行)
def cmd_power_off(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.stop_sys()
    return response
```
**功能**: 控制机器人系统上电/断电

---

### 3.3 使能控制服务

**服务名**:
- `/system_service/enable` (第22行)
- `/system_service/disable` (第23行)

**实现代码**:
```python
# 使能 (第43-45行)
def cmd_enable(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.start_sys()
    return response

# 去使能 (第47-49行)
def cmd_disable(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.stop_sys()
    return response
```
**功能**: 使能/去使能机器人伺服系统

---

### 3.4 运动控制服务

**服务名**:
- `/system_service/pause_motion` (第27行)
- `/system_service/resume_motion` (第28行)
- `/system_service/abort_motion` (第29行)

**实现代码**:
```python
# 暂停运动 (第51-53行)
def cmd_pause_motion(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.pause()
    return response

# 恢复运动 (第55-57行)
def cmd_resume_motion(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.resume()
    return response

# 中止运动 (第59-61行)
def cmd_abort_motion(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.stop()
    return response
```
**功能**: 控制机器人运动的暂停、恢复和中止

---

### 3.5 示教模式服务

**服务名**:
- `/system_service/entry_teach_mode` (第24行)
- `/system_service/exit_teach_mode` (第25行)

**实现代码**:
```python
# 进入示教模式 (第63-65行)
def cmd_entry_teach_mode(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.teach_mode()
    return response

# 退出示教模式 (第67-69行)
def cmd_exit_teach_mode(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.end_teach_mode()
    return response
```
**功能**: 进入/退出手动示教模式（可手动拖动机械臂）

---

### 3.6 关机服务

**服务名**: `/system_service/turn_off_robot` (第26行)

**实现代码**:
```python
# 第71-73行
def cmd_turn_off_robot(self, request: Empty.Request, response: Empty.Response):
    self.lebai_robot_.powerdown()
    return response
```
**功能**: 关闭机器人控制器

---

## 4. 抓取服务 (grab_demo)

### 4.1 GrabObject - 物体抓取服务

**服务定义文件**: `src/grab_demo/srv/GrabObject.srv`

```
string obj_link     # 目标物体的TF链接名
---
bool success        # 执行是否成功
string message      # 返回消息
```

**实现位置**: `src/grab_demo/src/grab_service_node.cpp`

**服务注册**: `/obj_grab_service` (第41行)

**实现代码**:

```cpp
// 第54-160行
void obj_grab::start_grab(
    const std::shared_ptr<grab_demo::srv::GrabObject::Request> req,
    std::shared_ptr<grab_demo::srv::GrabObject::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "开始抓取任务，目标TF: %s", req->obj_link.c_str());

    // 1. 张开夹爪
    p_request->val=100;
    auto result_=lebai_pclient->async_send_request(p_request);

    // 2. 设置TF缓冲区和监听器
    tf2_ros::Buffer buf(this->get_clock());
    tf2_ros::TransformListener listener(buf);

    // 3. 检查TF变换是否可用
    if (!buf.canTransform("base_link", req->obj_link, rclcpp::Time(0),
                          rclcpp::Duration::from_seconds(5.0))) {
        RCLCPP_ERROR(this->get_logger(), "无法获取 TF 变换: base_link -> %s",
                     req->obj_link.c_str());
        res->success = false;
        res->message = "TF transform not available";
        return;
    }

    // 4. 获取物体位姿
    geometry_msgs::msg::TransformStamped tfs;
    try {
        tfs = buf.lookupTransform("base_link", req->obj_link, rclcpp::Time(0),
                                  rclcpp::Duration::from_seconds(5.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF 查找失败: %s", ex.what());
        res->success = false;
        res->message = std::string("TF lookup failed: ") + ex.what();
        return;
    }

    // 5. 获取当前位姿并设置目标位姿
    geometry_msgs::msg::Pose cur_pose=move_group->getCurrentPose(end_link).pose;
    target_pose.position.x=tfs.transform.translation.x;
    target_pose.position.y=tfs.transform.translation.y;
    target_pose.position.z=tfs.transform.translation.z;  // 已移除Z轴补偿

    // 保持当前姿态
    target_pose.orientation.x = cur_pose.orientation.x;
    target_pose.orientation.y = cur_pose.orientation.y;
    target_pose.orientation.z = cur_pose.orientation.z;
    target_pose.orientation.w = cur_pose.orientation.w;

    // 6. 移动到观察点
    move_group->setNamedTarget("look");
    moveit::core::MoveItErrorCode move_result = move_group->move();

    // 7. 规划到目标位置
    move_group->setStartStateToCurrentState();
    sleep(1);
    move_group->setPoseTarget(target_pose);
    move_group->setPlanningTime(30);
    moveit::core::MoveItErrorCode plan_result = move_group->plan(my_plan);

    // 8. 执行运动
    if(plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        moveit::core::MoveItErrorCode execute_result = move_group->execute(my_plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            res->success = true;
            res->message = "Successfully moved to target";
        } else {
            res->success = false;
            res->message = "Execution failed";
        }
    } else {
        res->success = false;
        res->message = "Planning failed";
        return;
    }

    // 9. 闭合夹爪并返回观察点
    sleep(1);
    p_request->val=0;
    result_=lebai_pclient->async_send_request(p_request);
    sleep(1);
    move_group->setNamedTarget("look");
    move_group->move();
}
```

**功能说明**:
- 基于视觉的物体抓取服务
- 使用TF2获取物体位置
- 使用MoveIt进行运动规划和执行
- 调用IO服务控制夹爪开合
- 完整的错误处理和日志记录

**工作流程**:
1. 张开夹爪准备抓取
2. 通过TF获取目标物体的空间位置
3. 移动到预设的观察点"look"
4. 规划到物体位置的运动轨迹
5. 执行运动到达物体位置
6. 闭合夹爪抓取物体
7. 返回观察点

**依赖服务**:
- `/io_service/set_gripper_position` - 夹爪控制

---

## 服务调用示例

### Python 客户端示例

```python
import rclpy
from rclpy.node import Node
from lebai_interfaces.srv import SetGripper

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.client = self.create_client(SetGripper, '/io_service/set_gripper_position')

    def send_request(self, position):
        request = SetGripper.Request()
        request.val = position
        future = self.client.call_async(request)
        return future

# 使用
node = GripperClient()
future = node.send_request(100.0)  # 张开夹爪
```

### C++ 客户端示例

```cpp
auto client = this->create_client<lebai_interfaces::srv::SetGripper>(
    "/io_service/set_gripper_position");

auto request = std::make_shared<lebai_interfaces::srv::SetGripper::Request>();
request->val = 100.0;  // 张开夹爪

auto result = client->async_send_request(request);
```

### 命令行调用示例

```bash
# 调用夹爪服务
ros2 service call /io_service/set_gripper_position lebai_interfaces/srv/SetGripper "{val: 100.0}"

# 调用急停服务
ros2 service call /system_service/emergency_stop std_srvs/srv/Empty

# 调用运动服务
ros2 service call /motion_service/move_joint lebai_interfaces/srv/MoveJoint "{is_joint_pose: true, joint_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], common: {acc: 1.0, vel: 1.0, time: 0.0, radius: 0.0}}"

# 调用抓取服务
ros2 service call /obj_grab_service grab_demo/srv/GrabObject "{obj_link: 'object_1'}"
```

---

## 服务依赖关系图

```
grab_service_node (抓取服务)
  ├─> /io_service/set_gripper_position (夹爪控制)
  ├─> MoveIt (运动规划)
  └─> TF2 (坐标变换)

io_service (IO服务)
  └─> LebaiRobot SDK

motion_service (运动服务)
  └─> LebaiRobot SDK

system_service (系统服务)
  └─> LebaiRobot SDK
```

---

## 总结

### 服务统计

- **IO服务**: 9个服务接口
  - SetDO: 3个（机器人/扩展/法兰）
  - SetAO: 2个（机器人/扩展）
  - SetAMode: 2个（输入/输出模式）
  - SetGripper: 2个（位置/力度）

- **运动服务**: 2个服务接口
  - MoveJoint: 关节运动
  - MoveLine: 直线运动
  - MoveCircle: 圆弧运动（未实现）

- **系统服务**: 11个服务接口
  - 急停、上下电、使能、示教模式、运动控制等

- **抓取服务**: 1个服务接口
  - GrabObject: 完整的物体抓取流程

### 关键文件路径

| 功能 | 服务定义 | 实现文件 |
|------|----------|----------|
| IO服务 | `lebai_interfaces/srv/*.srv` | `io_service/io_service_interface.py` |
| 运动服务 | `lebai_interfaces/srv/Move*.srv` | `motion/tp_trajectory_handler.py` |
| 系统服务 | `std_srvs/srv/Empty.srv` | `system_service/system_service_interface.py` |
| 抓取服务 | `grab_demo/srv/GrabObject.srv` | `grab_demo/src/grab_service_node.cpp` |

---

**文档创建时间**: 2026-01-01
**分支**: Jetson
**版本**: 1.0