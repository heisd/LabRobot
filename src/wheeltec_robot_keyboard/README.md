# wheeltec_robot_keyboard

WheelTec S300 机器人 ROS2 键盘遥控包,通过捕获终端按键实时发布 `cmd_vel`,实现 WASD/方向键等控制。

## 概述

纯 Python 实现的键盘遥控节点,通过 `tty`+`termios` 在终端中无回显地读取按键,然后转换为 `geometry_msgs/Twist` 发布到 `cmd_vel`。

## 目录结构

```
wheeltec_robot_keyboard/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/wheeltec_robot_keyboard
├── wheeltec_robot_keyboard/
│   ├── __init__.py
│   └── wheeltec_keyboard.py     # 主节点实现
└── test/                         # 代码风格测试
```

## 依赖项

- buildtool: `ament_python`
- depend: `rclpy`、`geometry_msgs`
- test_depend: `ament_copyright`、`ament_flake8`、`ament_pep257`、`python3-pytest`

## 节点说明

### `wheeltec_keyboard`(可执行文件:`wheeltec_keyboard`)

发布:
- `cmd_vel` (geometry_msgs/Twist) — 速度指令,默认 QoS 为 keep_last(10)

按键映射(摘自源码):
| 按键 | 动作 |
| --- | --- |
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 左转 / 右转 |
| `u`/`o`/`m`/`.` | 斜向移动 |
| `J` / `L` | 全向平移(纯左/右) |
| `q` / `z` | 整体加速 / 减速 10% |
| `w` / `x` | 仅线速度加 / 减 10% |
| `e` / `c` | 仅角速度加 / 减 10% |
| `space` 或 `k` | 急停 |
| `b` | 切换全向模式 |
| `Ctrl+C` | 退出 |

内部初始速度:`speed=0.2 m/s`,`turn=1.0 rad/s`。

## 编译与运行

```bash
colcon build --packages-select wheeltec_robot_keyboard
source install/setup.bash
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
```

## 使用示例

```bash
# 启动底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 在新终端启动键盘遥控
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
# 终端进入按键监听模式,使用 i/j/k/l/, 控制机器人
```

## 注意事项

1. 必须在前台终端中运行,远程 ssh 时建议加 `-t` 分配伪终端。
2. 退出时若终端回显异常,可执行 `reset` 恢复。
3. 与其他发布 `cmd_vel` 的节点同时运行时会互相覆盖,建议关闭其他遥控。
4. 全向移动按键(`J` / `L` / `b` 切换)仅对支持麦克纳姆轮等全向底盘的车型有效。
