# 乐白机械臂 Web Dashboard 使用说明

`dashboard` 节点是一个网页版机械臂控制面板，**完全对齐 SDK 暴露的 ROS2 接口**：
订阅 `robot_state` 的状态话题做显示，按钮调用 `system_service` / `io_service` /
`motion_service` 的服务。

- 仅用 Python 标准库 `http.server`，**无额外 pip 依赖**，适合 Jetson。
- 纯网页形式，**不弹任何本地窗口**（headless 安全），从笔记本浏览器远程访问即可。

## 一、对齐的接口

| 类别 | 接口 | 用途 |
|------|------|------|
| 订阅 | `/joint_states` (sensor_msgs/JointState) | 关节角度/速度显示 |
| 订阅 | `/robot_status` (lebai_interfaces/RobotStatus) | 急停/上电/可运动/运动中/错误/模式 |
| 订阅 | `/io_status` (lebai_interfaces/IOStatus) | 数字/模拟 IO 显示 |
| 订阅 | `/gripper_status` (lebai_interfaces/GripperStatus) | 夹爪位置/力度 |
| 系统服务 | `/system_service/{power_on,power_off,enable,disable,pause_motion,resume_motion,abort_motion,entry_teach_mode,exit_teach_mode,emergency_stop,turn_off_robot}` (std_srvs/Empty) | 系统级控制按钮 |
| IO 服务 | `/io_service/set_gripper_position`、`/io_service/set_gripper_force` (SetGripper) | 夹爪控制 |
| IO 服务 | `/io_service/set_robot_do` (SetDO)、`/io_service/set_robot_ao` (SetAO) | 数字/模拟输出 |
| 运动服务 | `/motion_service/move_joint` (MoveJoint) | 关节运动（会真实移动机械臂） |

> 命令命名空间都做成了参数（`system_service_ns` / `io_service_ns` / `motion_service_ns`），
> 默认 `/system_service`、`/io_service`、`/motion_service`，和驱动节点名一致。

## 二、编译

```bash
cd ~/lebai
colcon build --packages-select lebai_driver
source install/setup.bash
```

## 三、运行

Dashboard 本身不直接连机器人，它调用的是驱动各节点的服务，所以**先启动驱动**：

```bash
ros2 launch lebai_driver robot_state.launch.py
ros2 launch lebai_driver io_service.launch.py
ros2 launch lebai_driver system_service.launch.py
ros2 launch lebai_driver motion.launch.py        # 需要关节运动时
```

再启动 Dashboard：

```bash
ros2 launch lebai_driver dashboard.launch.py            # 默认端口 8080
# 或自定义端口
ros2 launch lebai_driver dashboard.launch.py http_port:=9000
# 或直接 run
ros2 run lebai_driver dashboard --ros-args -p http_port:=8080
```

然后在浏览器打开（Jetson 本机或局域网其它电脑）：

```
http://<Jetson-IP>:8080/
```

## 四、界面功能

顶部导航栏有两个页面：**监控与控制** 和 **功能启动**。

### 4.1 监控与控制

- **机器人状态**：急停 / 上电 / 可运动 / 运动中 / 错误 / 错误码 / 模式（每 0.5s 刷新）。
- **夹爪状态**：当前位置、力度。
- **IO 状态**：机器人 DI/DO、AI、法兰 DI、扩展 DI。
- **关节状态**：各关节角度（rad 和 °）、速度。
- **系统控制**：上电/断电/使能/去使能/暂停/恢复/中止/进入退出示教/急停/关机。
  危险操作（断电、急停、去使能、关机）会弹二次确认。
- **夹爪控制**：设置位置（0 闭合 ~ 100 张开）、设置力度。
- **数字输出 DO**：指定引脚置 ON/OFF。
- **关节运动**：填 6 个关节角（rad）+ acc/vel，点"执行"做 move_joint；
  "填入当前关节角"会把实时关节角填进输入框。**此操作会真实移动机械臂，有二次确认。**

### 4.2 功能启动

一键启动/停止预定义的功能（Dashboard 以子进程方式 `ros2 launch`，并跟踪运行状态）：

| 分组 | 功能 | 实际命令 |
|------|------|----------|
| 视觉抓取 | YOLO 抓取 | `ros2 launch grab_demo yolo_grab.launch.py` |
| 视觉抓取 | HSV/颜色 抓取 | `ros2 launch grab_demo color_grab.launch.py` |
| 视觉抓取 | ArUco 抓取 | `ros2 launch grab_demo aruco_grab.launch.py` |
| 视觉抓取 | 手眼标定 | `ros2 launch grab_demo hand_eye.launch.py` |
| 机器人驱动 | robot_state / io_service / system_service / motion | `ros2 launch lebai_driver *.launch.py` |
| 运动规划 | MoveIt (lm3) | `ros2 launch lebai_lm3_moveit_config lm3.launch.py` |

每项有 **启动 / 停止 / 日志** 按钮，绿点表示运行中（显示 pid 和运行时长）。

**冲突自动规避（前端 JS 实现）**：每个任务声明它占用的资源（`camera` / `robot_state` /
`motion` / `io_service` / `system_service` / `moveit` / `grab`），两个任务只要资源有交集就算冲突。
界面会**自动禁用**会与"运行中任务"冲突的"启动"按钮，并标注 `⚠ 与运行中的【…】冲突`。
例如：

- 视觉抓取（YOLO/HSV/ArUco/手眼）内部已包含相机 + 全套驱动 + MoveIt，所以
  **它们互相禁用**，且一旦启动其一，"机器人驱动""MoveIt (lm3)" 也会被禁用；
- 反过来，先启动了 `robot_state` / `motion` / `MoveIt` 等，视觉抓取也会被禁用；
- `robot_state` / `io_service` / `system_service` / `motion` 之间资源不重叠，**可以共存**。

要换一套功能时，先"停止"正在运行的，冲突按钮会自动恢复可用。

> 注意：
> - 停止时 Dashboard 会对整个进程组发 `SIGINT` 优雅关闭（ros2 launch 会连带关掉它拉起的所有节点），
>   超时未退出再强制结束。
> - 命令是**固定白名单**（在 `dashboard_node.py` 的 `LAUNCH_TASKS` 里定义），
>   网页端不能执行任意命令；要增删功能改这个列表即可。
> - 任务日志写在 `${TMPDIR:-/tmp}/lebai_dashboard_logs/<id>.log`。
> - **前提**：启动 Dashboard 的终端已 `source install/setup.bash`，否则子进程找不到 `ros2`/功能包。

## 五、参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `http_host` | `0.0.0.0` | 监听地址，`0.0.0.0` 允许局域网访问 |
| `http_port` | `8080` | 网页端口 |
| `system_service_ns` | `/system_service` | 系统服务命名空间 |
| `io_service_ns` | `/io_service` | IO 服务命名空间 |
| `motion_service_ns` | `/motion_service` | 运动服务命名空间 |

## 六、HTTP API（也可脚本调用）

- `GET /api/status` → 返回当前状态 JSON。
- `POST /api/command` → 执行命令，body 示例：
  - 系统：`{"type":"system","name":"power_on"}`
  - 夹爪位置：`{"type":"gripper_position","val":100}`
  - 夹爪力度：`{"type":"gripper_force","val":50}`
  - 数字输出：`{"type":"set_do","pin":0,"value":true}`
  - 模拟输出：`{"type":"set_ao","pin":0,"value":3.3}`
  - 关节运动：`{"type":"move_joint","joint_pose":[0,0,0,0,0,0],"acc":1.0,"vel":1.0}`
- `GET /api/tasks` → 功能启动任务的状态列表。
- `POST /api/task` → 启停功能，body：`{"id":"yolo_grab","action":"start"}`（`action` 为 `start`/`stop`）。
- `GET /api/task_log?id=yolo_grab` → 返回该任务最近的日志（纯文本）。

```bash
curl http://localhost:8080/api/status
curl -X POST http://localhost:8080/api/command -d '{"type":"system","name":"enable"}'
# 一键启动 YOLO 抓取 / 查看日志 / 停止
curl -X POST http://localhost:8080/api/task -d '{"id":"yolo_grab","action":"start"}'
curl "http://localhost:8080/api/task_log?id=yolo_grab"
curl -X POST http://localhost:8080/api/task -d '{"id":"yolo_grab","action":"stop"}'
```

## 七、说明

- 命令均为非阻塞下发（`call_async`），按钮点完即返回"已发送"，实际结果以状态面板为准。
- 若某服务未就绪，会提示"服务未就绪（对应节点是否已启动？）"。
- 节点退出（Ctrl+C）时会自动关闭 HTTP 服务。
