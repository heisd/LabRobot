# wheeltec_path_follow

WheelTec 机器人路径录制 / 路径跟随包,用于沿事先录制好的路径自动行驶,常用于巡线巡逻、固定路径搬运等场景。

## 概述

由两部分组成:
1. **路径录制**(`save_path` C++ 节点):在用键盘/手柄遥控机器人沿期望轨迹行驶的同时,把 `odom` 中的位姿连续写入文本文件。
2. **路径跟随**(`follow_path.py` Python 脚本):读取保存的文件,通过 Nav2 `NavigateToPose` 或 `FollowPath` 接口将机器人沿轨迹回放。

## 目录结构

```
wheeltec_path_follow/
├── CMakeLists.txt
├── package.xml
├── path/
│   └── wheeltec_path                  # 默认录制的路径文本
├── src/
│   └── save_path.cpp                  # 录制节点
├── scripts/
│   └── follow_path.py                 # 跟随脚本
└── launch/
    ├── save_path.launch.py
    └── follow_path.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`rclpy`、`geometry_msgs`、`nav_msgs`、`tf2`、`tf2_ros`、`nav2_msgs`

## 节点说明

### `save_path`(C++,可执行文件:`save_path`)

发布:
- `followpath` (nav_msgs/Path) — 实时可视化已录制路径(供 RViz 显示)。

订阅:
- 通过 TF 监听 `odom -> base_footprint`,读取机器人当前位姿。

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `pathfilename` | `/home/wheeltec/wheeltec_ros2/src/wheeltec_path_follow/path/wheeltec_path` | 输出路径文本文件 |

运行后会一边遥控一边将位姿点追加保存到指定文件。

### `follow_path.py`(Python 脚本)

读取 `pathfilename` 指定的文件,把每个位姿点封装为 `PoseStamped` 发送到 Nav2 的导航 action,实现回放。

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `pathfilename` | `/home/wheeltec/wheeltec_ros2/src/wheeltec_path_follow/path/wheeltec_path` | 待回放的路径文件 |
| `run_in_loop` | `True` | 是否循环跟随 |

## 启动文件

- **`save_path.launch.py`** — 启动 `save_path` 节点;可指定输出文件位置。
- **`follow_path.launch.py`** — 启动 `follow_path.py`,可指定文件位置和是否循环。

## 编译与运行

```bash
colcon build --packages-select wheeltec_path_follow
source install/setup.bash
```

### 录制路径

```bash
# 终端1:启动底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端2:启动录制
ros2 launch wheeltec_path_follow save_path.launch.py \
    pathfilename:=$HOME/my_path.txt

# 终端3:用键盘遥控走完期望路径
ros2 run wheeltec_robot_keyboard wheeltec_keyboard

# Ctrl+C 结束录制
```

### 回放跟随

```bash
# 启动 Nav2 (确保已有地图并完成定位)
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

# 启动跟随
ros2 launch wheeltec_path_follow follow_path.launch.py \
    pathfilename:=$HOME/my_path.txt \
    run_in_loop:=True
```

## 注意事项

1. 默认参数路径写死为 `/home/wheeltec/wheeltec_ros2/...`,部署到其他用户家目录时务必通过参数重写。
2. 路径文件较大时回放过程中会消耗内存,建议合理控制采样点数量。
3. `follow_path.py` 通过 Nav2 action 调用,因此必须先有正确定位与有效地图。
4. 录制时机器人 odom 漂移会直接反映在路径上,建议在打滑较少的地面录制。
