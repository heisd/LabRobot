# wheeltec_robot_rrt

WheelTec 机器人的 RRT(Rapidly-exploring Random Tree)自主探索包,集成行为树(Behavior Tree)、Nav2 与 SLAM,实现"未知环境自主建图 + 寻找/抓取彩色物体"等复杂任务。

## 概述

包内包含三大类功能:

1. **RRT 自主探索**:基于 `rrt_exploration` 实现全局/局部 RRT 采样、前沿点聚类(mean_shift)、任务分配(assigner),把未知前沿作为 Nav2 目标点完成全自主建图。
2. **基于行为树的任务**:在 BT 中加入 `find_coloured_box`、`approach_coloured_box`、`pick_coloured_box` 等自定义节点,用于"找箱子 + 抓箱子"任务。
3. **Nav2 启动包装**:基于 Nav2 官方 launch 文件改写的 bringup/localization/navigation/slam launch,使探索/导航/SLAM 可独立或组合启动。

## 目录结构

```
wheeltec_robot_rrt2/
├── CMakeLists.txt
├── package.xml
├── behaviour_trees/                # 行为树 XML
├── config/
│   ├── nav2_params.yaml            # Nav2 参数
│   ├── exploration_params.yaml     # RRT 探索参数
│   ├── ekf.yaml                    # EKF 参数
│   └── rtabmap.yaml                # 视觉 SLAM(rtabmap)参数
├── launch/
│   ├── navigation/
│   │   ├── nav2_bringup_launch.py
│   │   ├── nav2_localization_launch.py
│   │   ├── nav2_navigation_launch.py
│   │   └── nav2_slam_launch.py
│   └── rrt_exploration/
│       ├── rrt_exploration.launch.py
│       ├── rrt_assigner.launch.py
│       ├── action_servers.launch.py
│       ├── wheeltec_rrt_slam.launch.py
│       └── visual_slam_start.py
├── scripts/
│   ├── robot_navigator.py          # Nav2 BasicNavigator 封装
│   ├── nav_to_pose_demo.py         # 简单导航 demo
│   ├── boundary_publisher.py       # 发布探索边界
│   └── box_picker.py               # 抓箱任务客户端
└── src/
    ├── rrt_exploration/            # RRT 探索 C++ 实现
    │   ├── global_rrt.cpp / local_rrt.cpp
    │   ├── filter.cpp / mean_shift.cpp
    │   ├── assigner.cpp / robot.cpp
    │   └── utils.cpp / wait_for_fin.cpp / mtrand.cpp
    ├── bt_plugins/                 # 行为树自定义节点
    │   ├── find_coloured_box.cpp
    │   ├── approach_coloured_box.cpp
    │   └── pick_coloured_box.cpp
    ├── find_robot_action_server.cpp
    ├── pick_robot_action_server.cpp
    ├── robot_picker.cpp
    └── robot_pose_publisher.cpp
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`rclcpp_action`、`rclpy`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`、`tf2`、`tf2_ros`、`nav2_msgs`、`nav2_behavior_tree`、`behaviortree_cpp_v3`、`wheeltec_rrt_msg`、`yaml-cpp`

## 节点说明

### RRT 探索类节点(C++)

- **`global_rrt_detector`** — 全局 RRT 前沿点检测,发布候选前沿点。
- **`local_rrt_detector`** — 局部 RRT 前沿点检测,提高响应速度。
- **`filter`** — 对候选前沿点做聚类(`mean_shift`)和有效性过滤。
- **`assigner`** — 任务分配,将过滤后的目标分配给可用机器人并通过 Nav2 NavigateToPose 派发。
- **`wait_for_fin`** — 检测探索结束条件。

### 行为树自定义节点

- `FindColouredBox`、`ApproachColouredBox`、`PickColouredBox` — 注册为 BehaviorTreeCpp v3 节点,用于行为树脚本。

### Action 服务端

- `find_robot_action_server`、`pick_robot_action_server` — 分别为 `wheeltec_rrt_msg/FindColouredBox`、`PickColouredBox` 提供服务端实现。

### Python 工具脚本

- `robot_navigator.py` — `BasicNavigator` 类,封装 Nav2 `navigate_to_pose`、`FollowWaypoints` action client。
- `boundary_publisher.py` — 在 RViz 中通过点击发布探索边界多边形。
- `box_picker.py` — 抓箱任务客户端。

## 启动文件

### `launch/navigation/`

- `nav2_bringup_launch.py` — Nav2 完整协议栈一键启动
- `nav2_localization_launch.py` — 仅 AMCL 定位
- `nav2_navigation_launch.py` — 仅 Planner/Controller/BT
- `nav2_slam_launch.py` — Nav2 + slam_toolbox

### `launch/rrt_exploration/`

- `rrt_exploration.launch.py` — 启动 `global_rrt_detector` + `local_rrt_detector` + `filter`
- `rrt_assigner.launch.py` — 启动 `assigner` 任务分配
- `action_servers.launch.py` — 启动 find/pick action servers
- `wheeltec_rrt_slam.launch.py` — 顶层组合 launch:底盘 + SLAM + 探索
- `visual_slam_start.py` — 启动 rtabmap 视觉 SLAM

## 参数配置

- `config/exploration_params.yaml` — RRT 步长、eta、地图话题、坐标系等
- `config/nav2_params.yaml` — Nav2 协议栈所有参数
- `config/rtabmap.yaml` — rtabmap 视觉 SLAM 参数
- `config/ekf.yaml` — robot_localization EKF 参数

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_robot_rrt
source install/setup.bash

# 一键启动:底盘 + SLAM + RRT 探索
ros2 launch wheeltec_robot_rrt wheeltec_rrt_slam.launch.py

# 在 RViz 中用 Publish Point 工具点击 5 个点界定探索区域边界
```

## 使用示例

```bash
# 1. 启动机器人 + 雷达
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py

# 2. 启动 SLAM
ros2 launch wheeltec_robot_rrt nav2_slam_launch.py

# 3. 启动探索算法
ros2 launch wheeltec_robot_rrt rrt_exploration.launch.py
ros2 launch wheeltec_robot_rrt rrt_assigner.launch.py

# 4. 在 RViz 用 Publish Point 工具点击 5 个点:
#    前 4 个点构成探索区域多边形,第 5 个点为机器人初始点
```

## 注意事项

1. 包名为 `wheeltec_robot_rrt`(注意目录名带 `2`)。
2. 探索算法对计算资源敏感,推荐 NUC/Jetson 以上算力。
3. 行为树节点 `pick_coloured_box` 假设机械臂/夹爪等外设可用,纯底盘机器人请屏蔽相关 BT 子树。
4. 探索区域多边形必须 **闭合且包含机器人当前位置**,否则 assigner 会卡住。
5. 多机器人探索时需要把 `assigner` 改成多 namespace。
