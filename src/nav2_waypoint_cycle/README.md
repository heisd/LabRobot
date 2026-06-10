# nav2_waypoint_cycle

WheelTec 机器人 Nav2 多点循环导航包,通过订阅 RViz `Publish Point` 工具发布的 `/clicked_point` 收集途经点,然后依次发送目标位姿给 Nav2 自动巡航。

## 概述

简洁的 Python 节点。用户在 RViz 中通过 `Publish Point` 工具点击若干个点,节点会逐点发布 `/goal_pose` 调用 Nav2,并在所有点完成后从头开始循环。该包是 `wheeltec_nav2` 的子模块,二者通常配套使用。

## 目录结构

```
nav2_waypoint_cycle/
├── package.xml
├── setup.py / setup.cfg
├── resource/
└── nav2_waypoint_cycle/
    ├── __init__.py
    └── waypoint_cycle.py   # 主节点
```

## 依赖项

- buildtool: `ament_python`
- depend: `rclpy`、`geometry_msgs`、`visualization_msgs`、`action_msgs`(`GoalStatusArray`)、`std_msgs`

## 节点说明

### `waypoint_cycle`(可执行文件:`nav2_waypoint_cycle`)

订阅:
- `/clicked_point` (geometry_msgs/PointStamped) — 由 RViz `Publish Point` 工具发布的点,用于追加航点。
- `navigate_to_pose/_action/status` (action_msgs/GoalStatusArray) — 监听 Nav2 NavigateToPose 动作的执行状态,完成后自动发送下一个航点。

发布:
- `/goal_pose` (geometry_msgs/PoseStamped) — 发送给 Nav2 的下一个导航目标。
- `/path_point` (visualization_msgs/MarkerArray) — 航点的 RViz 可视化标记。

工作流程:
1. 用户在 RViz 中多次点击 `Publish Point`,节点把每个点保存为目标。
2. 每点击一次都会刷新 `/path_point` 上的 Marker。
3. 收到第一个点后立刻发布到 `/goal_pose`,Nav2 开始执行。
4. 监听 `navigate_to_pose/_action/status`,当某段导航返回 `STATUS_SUCCEEDED` 时,自动发布下一段。
5. 所有点循环结束后回到第一个点,无限循环。

## 编译与运行

```bash
colcon build --packages-select nav2_waypoint_cycle
source install/setup.bash
ros2 run nav2_waypoint_cycle nav2_waypoint_cycle
```

## 使用示例

```bash
# 启动导航(已包含 waypoint_cycle 节点)
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

# 在 RViz 中:
#   1) 用 "2D Pose Estimate" 给定初始位姿
#   2) 选择 "Publish Point" 工具,依次点击若干个目标点
#   3) 机器人开始按点循环导航
```

## 注意事项

1. 必须先有正在运行的 Nav2(提供 `navigate_to_pose` action),否则导航不会触发。
2. 节点目前没有持久化目标点,重启后丢失,需要重新点击。
3. 默认从第一个点开始无限循环,如果只想跑一遍可修改 `pose_callback` 逻辑。
4. 若 Nav2 长时间到达失败,需手动重置初始位姿。
