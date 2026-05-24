# bodyreader

基于奥比中光(Orbbec)Astra 深度相机 SDK 的人体骨骼识别 + 跟随 + 交互节点,实现"看到人 → 锁定 → 跟随 → 手势/姿态交互"完整功能。

## 概述

包内含多个功能节点:
- **`bodyreader`**(main.cpp):从 Astra 深度相机读取数据,运行 SDK 的 Body Tracking 模块,输出 `Bodylist`、人体掩膜、可选 RGB 图。
- **`follower`**(follower.cpp):订阅 `Bodyposture`,通过 PID 控制 `cmd_vel` 跟随目标人体。
- **`interaction`**(interaction.cpp):识别人体手势/姿态,触发对应交互(语音、动作等)。
- **`feedback`**(feedback.cpp):任务反馈/语音回应。
- **`image_trans`**(image_trans.cpp):图像转发/格式转换。

## 目录结构

```
bodyreader/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   ├── main.cpp              # 主识别节点
│   ├── bodydata_process.cpp  # 骨骼数据处理
│   ├── follower.cpp          # 跟随控制
│   ├── interaction.cpp       # 姿态交互
│   ├── feedback.cpp          # 反馈
│   └── image_trans.cpp       # 图像转换
├── msg/                      # 内部消息
├── lib/                      # Astra SDK 动态库
├── audio/                    # 语音反馈音频
├── scripts/
└── launch/
    ├── bodyfollow.launch.py
    ├── bodyinteraction.launch.py
    └── final.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`rclpy`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`cv_bridge`、`bodyreader_msg`、`OpenCV`、Astra/Orbbec Body Tracking SDK

## 节点说明

### `bodyreader_node`(main.cpp)

发布:
- `/bodylist` (bodyreader_msg/Bodylist) — 人体骨骼列表
- `/body/mask` (bodyreader_msg/Maskdata) — 人体掩膜
- `/image_raw` (sensor_msgs/Image) — 可选 RGB 图

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `rgb_stream` | `false` | 是否启用 RGB 流 |
| `body_stream` | `true` | 是否启用人体骨骼流 |

### `follower_node`(follower.cpp)

订阅:
- `/body_posture` (bodyreader_msg/Bodyposture) — 当前人体姿态
- `/mode` (std_msgs/Int8) — 跟随模式切换

发布:
- `/cmd_vel` (geometry_msgs/Twist) — 跟随速度指令

参数(PID 增益):
| 参数 | 默认值 |
| --- | --- |
| `bodyfollow_x_p` | `0.01` |
| `bodyfollow_x_d` | `0.01` |
| `bodyfollow_z_p` | `0.01` |
| `bodyfollow_z_d` | `0.01` |
| `mode` | `2` |

### `interaction_node`、`feedback_node`、`image_trans_node`

姿态交互、语音反馈、图像中转节点,各自订阅 `Bodylist` / `Bodyposture` 等内部消息工作。

## 启动文件

- **`bodyfollow.launch.py`** — 启动 `bodyreader` + `follower`,实现人体跟随。
- **`bodyinteraction.launch.py`** — 启动 `bodyreader` + `interaction` + `feedback`,实现姿态交互。
- **`final.launch.py`** — 完整功能(跟随 + 交互 + 反馈)。

## 编译与运行

```bash
colcon build --packages-up-to bodyreader
source install/setup.bash

# 人体跟随
ros2 launch bodyreader bodyfollow.launch.py
```

## 使用示例

```bash
# 0. 安装并测试 Astra Body Tracking SDK
# 1. 启动底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 2. 启动人体跟随
ros2 launch bodyreader bodyfollow.launch.py

# 3. 站在相机前 1.5m 处,机器人应根据人体姿态跟随
```

## 注意事项

1. 必须正确安装 Orbbec / Astra SDK,且 `lib/` 中的动态库与本机架构(x86_64 / aarch64)匹配。
2. 跟随时机器人会直接发布 `cmd_vel`,务必保留 `space` 急停手段。
3. PID 增益需根据实际场地/速度调试,默认参数较保守。
4. 多人同时进入视野时,SDK 会按 ID 锁定首个识别到的人,可结合 `Lockedcharrgb` 改进。
