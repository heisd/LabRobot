# wheeltec_rrt_msg

`wheeltec_robot_rrt` 自主探索 + 抓取任务所使用的 ROS2 接口包,包含消息、服务和动作三类自定义接口。

## 概述

纯接口包,无可执行节点。仅向 `wheeltec_robot_rrt` 等业务包提供统一的接口定义,实现 RRT 前沿点传递、形状删除、彩色物体抓取等任务调度。

## 目录结构

```
wheeltec_rrt_msg/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── PointArray.msg
├── srv/
│   └── DeleteShape.srv
└── action/
    ├── ChangePosition.action
    ├── FindColouredBox.action
    └── PickColouredBox.action
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `geometry_msgs`、`std_msgs`、`action_msgs`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 消息定义

### `msg/PointArray.msg`

```
geometry_msgs/Point[] points
```

数组形式的三维点,RRT 探索中用于发布候选前沿点集。

## 服务定义

### `srv/DeleteShape.srv`

```
# Request
string shape_type     # 形状类型/标识
---
# Response
bool result           # 是否删除成功
```

用于在地图/可视化中删除指定的标记形状。

## 动作定义

### `action/ChangePosition.action`

```
# Goal
float64 desired_x
float64 desired_y
---
# Result
bool is_complete
---
# Feedback
float64 current_x
float64 current_y
```

请求机器人移动到 `(desired_x, desired_y)`,过程中持续反馈当前位置。

### `action/FindColouredBox.action`

```
# Goal
string box_colour
---
# Result
geometry_msgs/PoseStamped box_pickup_position
---
# Feedback
# (空)
```

请求寻找指定颜色的箱体,返回可抓取位姿。

### `action/PickColouredBox.action`

```
# Goal
string box_colour
---
# Result
std_msgs/Empty result
---
# Feedback
# (空)
```

请求抓取指定颜色的箱体。

## 编译

```bash
colcon build --packages-select wheeltec_rrt_msg
source install/setup.bash

ros2 interface show wheeltec_rrt_msg/msg/PointArray
ros2 interface show wheeltec_rrt_msg/action/FindColouredBox
```

## 使用示例

```python
from wheeltec_rrt_msg.action import FindColouredBox
from rclpy.action import ActionClient

client = ActionClient(node, FindColouredBox, 'find_coloured_box')
client.wait_for_server()
goal = FindColouredBox.Goal()
goal.box_colour = 'red'
client.send_goal_async(goal)
```

## 仪表盘可视化

`wheeltec_dashboard` 的 **建图 → RRT 自主探索** 子页已接入本接口：在面板
地图上点 5 个点（或一键方形边界）发布 `/clicked_point` 圈定探索区域，
并实时叠加显示 `/detected_frontiers` 与 `/filtered_goal_points`
（`wheeltec_rrt_msg/msg/PointArray`）的前沿/候选目标，`global_rrt` /
`local_rrt` / `filter` / `assigner` 四个探索节点在线状态同页判活。
详见 `src/wheeltec_dashboard/README.md`。

## 注意事项

1. 修改接口字段后必须重编译该包及所有依赖它的包。
2. 颜色字符串建议使用小写英文(`"red"`、`"green"`、`"blue"`)以与 BT 节点对应。
