# wheeltec_mic_ros2

WheelTec 6/4 麦克风阵列(基于科大讯飞 AIUI/MSC)的 ROS2 驱动与语音控制包,集成唤醒、命令词识别、在/离线 ASR、语音运动控制以及音频反馈。
注:这个只可以在我们的ARM机器上进行编译X64不兼容

## 概述

通过串口与麦克风阵列通讯,实现:
- **唤醒检测**:发布唤醒标志、麦克风指向角度。
- **命令词识别**:把"前进/后退/左转/跟随我"等口令转换为 `MotionControl` 消息。
- **运动控制桥接**:`motion_control` 节点把 `MotionControl` 转为 `cmd_vel`。
- **节点级反馈**:播放本地音频反馈用户语音指令的结果。
- **服务接口**:动态设置唤醒词、切换主麦克风等。

## 目录结构

```
wheeltec_mic_ros2/
├── CMakeLists.txt
├── package.xml
├── include/
├── lib/                              # 讯飞动态库
├── audio/                            # 本地反馈音频
├── feedback_voice/                   # 反馈语音资源
├── config/                           # 唤醒词/命令词/参数 yaml
├── tmp/                              # 运行临时文件
├── src/
│   ├── wheeltec_mic.cpp              # 串口驱动 + 唤醒/识别主节点
│   ├── call_recognition.cpp          # 唤醒(呼叫)识别
│   ├── command_recognition.cpp       # 命令词识别
│   ├── motion_control.cpp            # 语音 -> cmd_vel
│   ├── node_feedback.cpp             # 音频反馈
│   ├── voice_control.cpp             # 语音控制流程
│   └── ros2_service.cpp              # 服务实现(set_awake_word 等)
└── launch/
    ├── base.launch.py                # 完整启动(底盘 + 麦克风 + 控制)
    ├── mic_init.launch.py            # 仅麦克风初始化
    └── include/
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`std_msgs`、`geometry_msgs`、`sensor_msgs`、`serial`、`wheeltec_mic_msg`、讯飞 MSC/AIUI SDK、ALSA

## 节点说明

### `wheeltec_mic`(主驱动)

发布:
- `awake_flag` (std_msgs/Int8) — 唤醒状态
- `voice_flag` (std_msgs/Int8) — 语音命令状态
- `awake_angle` (std_msgs/UInt32) — 唤醒方向(度)
- `voice_words` (std_msgs/String) — 识别到的文字

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `usart_port_name` | `/dev/ttyCH343USB0` | 麦克风阵列串口 |
| `serial_baud_rate` | `115200` | 串口波特率 |

### `call_recognition` / `command_recognition`

调用讯飞 SDK 实现唤醒与命令词识别,识别后发布 `voice_flag` / `voice_words` / `MotionControl`。

### `motion_control`

订阅 `wheeltec_mic_msg/MotionControl`,转换为 `geometry_msgs/Twist`,发布到 `cmd_vel`。

### `node_feedback`

订阅唤醒 / 识别结果,播放 `feedback_voice/` 内的反馈语音。

### 服务

由 `ros2_service.cpp` 提供:
- `set_awake_word` (wheeltec_mic_msg/SetAwakeWord)
- `set_major_mic` (wheeltec_mic_msg/SetMajorMic)
- `switch_mic` (wheeltec_mic_msg/SwitchMic)
- `get_offline_result` (wheeltec_mic_msg/GetOfflineResult)
- `get_device_type` (wheeltec_mic_msg/GetDeviceType)

## 启动文件

- **`mic_init.launch.py`** — 仅启动麦克风阵列初始化(串口握手 + 配置)。
- **`base.launch.py`** — 完整启动:`wheeltec_mic` + `call_recognition` + `command_recognition` + `motion_control` + `node_feedback`。
- **`launch/include/`** — 子 launch 模块。

## 编译与运行

```bash
colcon build --packages-up-to wheeltec_mic_ros2
source install/setup.bash

# 完整启动
ros2 launch wheeltec_mic_ros2 base.launch.py
```

## 使用示例

```bash
# 1. 给麦克风阵列上电,确认 /dev/ttyCH343USB0 可见

# 2. 启动底盘 + 麦克风语音控制
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch wheeltec_mic_ros2 base.launch.py

# 3. 说出唤醒词(默认"小车小车"),再说"前进"/"左转"/"跟随我",
#    机器人会通过 motion_control 节点产生 cmd_vel
```

## 注意事项

1. 必须正确连接麦克风阵列且 udev 中映射到 `/dev/ttyCH343USB0`(或修改参数)。
2. 讯飞 SDK 需要 APPID 授权,在线服务需联网。
3. 离线命令词需先用 `set_awake_word`、`get_offline_result` 服务配置。
4. 与其他 `cmd_vel` 发布者(键盘 / joy / KCF 跟随)同时运行会冲突。
5. 反馈音频与系统默认音频设备共用,可能需在 `~/.asoundrc` 中指定 card。
