# wheeltec_mic_msg

WheelTec 6 麦克风阵列(科大讯飞方案)的 ROS2 消息 / 服务接口包。

## 概述

纯接口包(无可执行节点),为 `wheeltec_mic_ros2` 节点提供运动控制反馈、PCM 数据传输、离线唤醒/命令词配置等接口。

## 目录结构

```
wheeltec_mic_msg/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── MotionControl.msg
│   └── PcmMsg.msg
└── srv/
    ├── GetDeviceType.srv
    ├── GetOfflineResult.srv
    ├── SetAwakeWord.srv
    ├── SetMajorMic.srv
    └── SwitchMic.srv
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: `std_msgs`
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 消息定义

### `msg/MotionControl.msg`

```
float32 linear_x
float32 linear_y
float32 angular_z
int8    cmd_vel_flag       # 是否产生速度指令
int8    follow_flag        # 跟随标志
int8    goal_reached_flag  # 到达标志
```

由命令词识别 / 语音控制节点发布,通常被 `motion_control` 节点订阅以生成 `cmd_vel`。

### `msg/PcmMsg.msg`

```
int32      length
string[]   pcm_buf
```

承载 16kHz/16bit 单声道原始 PCM 音频(以 string 分块)。

## 服务定义

### `srv/SetAwakeWord.srv`

```
# Request
string text         # 唤醒词文本(如 "你好小车")
string awake_word   # 唤醒词标识
string threshold    # 阈值
---
# Response
string result
string fail_reason
```

### `srv/SetMajorMic.srv`

```
int8 mic_id      # 指定主麦克风编号
---
bool success
string result
```

### `srv/SwitchMic.srv`

```
string mic_name    # 切换到指定麦克风
---
string result
string fail_reason
```

### `srv/GetOfflineResult.srv`

```
# Request
int8 offline_recognise_start    # 启动 / 停止离线识别
int8 confidence_threshold       # 置信度阈值
int8 time_per_order             # 每条命令最长时长
---
bool success
string result
```

### `srv/GetDeviceType.srv`

```
---
bool success
string result
```

读取硬件设备类型。

## 编译

```bash
colcon build --packages-select wheeltec_mic_msg
source install/setup.bash
ros2 interface show wheeltec_mic_msg/msg/MotionControl
```

## 注意事项

1. `PcmMsg` 通过 `string[]` 传输二进制 PCM,在 Python 中要使用 `bytes` 转换。
2. 服务参数与底层 SDK 接口对应,修改前请参阅讯飞文档。
