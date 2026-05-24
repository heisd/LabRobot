# tts_make_ros2

基于讯飞(iFlyTek)在线 TTS SDK 的 ROS2 文字转语音节点,把指定文本合成为 WAV/PCM 音频文件并播放,支持配置发音人、语速、音调、采样率等。

## 概述

订阅文本/通过参数指定的文本通过讯飞 TTS API 在线合成语音,保存到本地并使用系统音频接口播放。常用于机器人语音播报、自动回应等场景。

## 目录结构

```
tts_make_ros2/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── include/
├── libs/                       # 讯飞 SDK 动态库
├── audio/                      # 输出音频
├── config/                     # 默认参数 yaml
├── src/
│   └── tts_make.cpp            # 主节点
└── launch/
    └── tts_make.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`std_msgs`、讯飞在线 TTS SDK(`libmsc.so`)、ALSA / PulseAudio 播放工具

## 节点说明

### `tts_make_node`

订阅 / 服务(根据源码版本可能略有差异):
- `tts_text` 参数或 `/tts_text` (std_msgs/String) — 待合成文本

主要参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `appid` | `""` | 讯飞控制台分配的 APPID |
| `source_path` | `""` | 输出音频保存路径 |
| `voice_name` | `""` | 发音人(`xiaoyan`、`aisxping` 等) |
| `tts_text` | `""` | 一次性合成的文本 |
| `volume` | `0` | 音量(0~100) |
| `pitch` | `0` | 音调 |
| `speed` | `0` | 语速 |
| `sample_rate` | `0` | 采样率(8000 / 16000) |
| `rdn` | `0` | 数字播报方式 |

## 启动文件

### `tts_make.launch.py`

启动 `tts_make` 节点,从 `config/` 读取 APPID、发音人等默认参数。

## 编译与运行

```bash
colcon build --packages-up-to tts_make_ros2
source install/setup.bash
ros2 launch tts_make_ros2 tts_make.launch.py
```

## 使用示例

```bash
# 通过参数方式播报
ros2 run tts_make_ros2 tts_make --ros-args \
    -p appid:=<YOUR_APPID> \
    -p voice_name:=xiaoyan \
    -p tts_text:="你好，我是 WheelTec 机器人"

# 也可通过修改 launch 中默认 tts_text 后启动
ros2 launch tts_make_ros2 tts_make.launch.py
```

## 注意事项

1. 必须在讯飞开放平台申请 `APPID` 并在 yaml / launch 中填入。
2. SDK 是 x86_64 / aarch64 不同版本,务必使用与本机匹配的 `libs/`。
3. 离线 TTS 需要单独购买授权,本包默认在线版本,需要联网。
4. 合成完成后会调用系统播放器(`aplay`),需确保音频设备可用。
5. 长文本会被 SDK 切片,首字延迟约几百毫秒,适合短播报。
