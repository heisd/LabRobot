# 抓取仲裁 (arm_arbiter) 使用说明

给机械臂加了一层**优先级仲裁**：**手动操作优先级最高**，可随时打断正在运行的自动抓取
（YOLO / KCF / HSV / VLM 都走同一套闭环抓取，受其管控）。

## 一、优先级与角色

| 优先级 | 角色 | 说明 |
|--------|------|------|
| 高 | **手动 (manual)** | Dashboard 的手动操作 / 点【手动接管】。可打断任何自动抓取 |
| 低 | **自动 (auto)** | YOLO/KCF/HSV/VLM 触发的闭环抓取 `closed_loop_grab_node` |

规则：
- **手动接管**会立刻**打断**正在运行的自动抓取，并在**释放前拒绝**新的自动抓取请求。
- 自动抓取**永远不会**打断手动；手动接管期间来的自动抓取请求会被直接拒绝。

## 二、组成

- **`arm_arbiter`（节点，`scripts/arm_arbiter_node.py`）** — 策略权威，发布状态、受理手动接管/释放：
  - 服务 `/arm_arbiter/manual_takeover` (std_srvs/Trigger)：手动接管，脉冲打断 + 暂停受理自动抓取。
  - 服务 `/arm_arbiter/manual_release` (std_srvs/Trigger)：释放，交还自动。
  - 话题 `/arm_arbiter/abort` (Bool)：打断信号（脉冲）。
  - 话题 `/arm_arbiter/manual_active` (Bool, latched)：手动是否占用（自动抓取据此拒绝新请求）。
  - 话题 `/arm_arbiter/state` (String)：当前控制权 `idle` / `manual`，供 Dashboard 显示。
- **`closed_loop_grab_node`** — 订阅 `abort` / `manual_active` 协作：
  - 用 **MultiThreadedExecutor + 独立回调组**，即使抓取正阻塞在 `move()` 里，收到打断也能在另一线程
    调用 `MoveGroupInterface::stop()` **真正中途停下**（而不是等这一段动作做完）；
  - 抓取过程中多处检查打断标志，被打断则立即让出并返回 `preempted by manual takeover`；
  - `manual_active=true` 时拒绝新的抓取请求。
- 仲裁节点已**随各抓取 launch 一起启动**（yolo_ros_grab / kcf_grab / color_grab / vlm_grab）。

> 解耦设计：自动抓取节点只**订阅** abort / manual_active 两个话题即可协作，不反向调用仲裁。
> 因此即便没启动 `arm_arbiter`，自动抓取也能降级照常运行（只是没有手动打断能力）。

## 三、在 Dashboard 里使用

监控页顶部新增 **【抓取仲裁】** 卡片：
- 显示当前控制权：`空闲(idle)` / `手动(manual)`；
- 【手动接管 (打断抓取)】：立刻打断正在运行的自动抓取，并暂停受理新的自动抓取；
- 【释放 (交还自动)】：恢复自动抓取受理。

此外，**手动关节运动**（监控页的 move_joint）会**自动先接管**再运动，确保手动操作即时优先。

## 四、命令行用法

```bash
# 手动接管(打断正在运行的自动抓取, 之后拒绝自动抓取)
ros2 service call /arm_arbiter/manual_takeover std_srvs/srv/Trigger

# 释放(交还自动)
ros2 service call /arm_arbiter/manual_release std_srvs/srv/Trigger

# 查看当前控制权
ros2 topic echo /arm_arbiter/state
```

## 五、典型流程

1. 启动任一抓取（如 `ros2 launch grab_demo yolo_ros_grab.launch.py`），`arm_arbiter` 随之启动；
2. 自动抓取运行中，你发现需要人工干预 → 点 Dashboard【手动接管】（或直接做手动关节运动）；
3. 机械臂**立即停止**当前自动动作，控制权变为 `manual`，期间自动抓取被拒绝；
4. 人工处理完毕 → 点【释放】，控制权回到 `idle`，自动抓取恢复。

> 注意：本仲裁管的是“谁来下发机械臂运动 + 能否打断自动抓取”。它不替代**硬件急停**；
> 真正的安全急停请用 `system_service` 的【急停】或物理急停按钮。
