# tts(目录名 `tts_make_ros2`)

> 注意:本目录名为 `tts_make_ros2`,但 `package.xml` 中 `<name>` 为
> **`tts`**,CMake `project()` 也是 `tts`,实际 `colcon` / `ros2 run`
> / `ros2 launch` 命令都必须使用 **`tts`** 这个包名,而不是目录名。

基于讯飞(iFlyTek)在线 TTS SDK 的 ROS2 文字转语音节点,把指定文本合成为音频并播放,支持配置发音人、语速、音调、采样率等。

## 概述

通过讯飞 MSC SDK 在线合成语音,保存到本地并播放。常用于机器人语音播报、自动回应等场景。

## 目录结构

```
tts_make_ros2/                  # 工作空间中的目录名
├── CMakeLists.txt              # project(tts)
├── package.xml                 # <name>tts</name>
├── LICENSE
├── include/
├── libs/                       # 讯飞 SDK 动态库(libmsc.so 等)
├── audio/                      # 输出音频
├── config/
│   └── tts_params.yaml         # 默认参数
├── src/
│   └── tts_make.cpp            # 主节点源码,编译为可执行 tts_node
└── launch/
    └── tts_make.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`std_msgs`、讯飞在线 TTS SDK(`libmsc.so`)、ALSA / PulseAudio 播放工具

## 节点说明

### `tts_node`(可执行,由 `src/tts_make.cpp` 编译生成)

来自 `CMakeLists.txt`:
```cmake
project(tts)
add_executable(${PROJECT_NAME}_node src/tts_make.cpp)   # => tts_node
```

参数(`config/tts_params.yaml` + launch 中传入):
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `appid` | `b22421bd` | 讯飞控制台分配的 APPID |
| `voice_name` | `xiaoyan` | 发音人(`xiaoyan` 等) |
| `sample_rate` | `16000` | 采样率(支持 8000 / 16000) |
| `volume` | `50` | 音量(0~100) |
| `pitch` | `50` | 音调(0~100) |
| `speed` | `50` | 语速(0~100) |
| `rdn` | `0` | 数字播报方式(0 数值优先 / 1 完全数值 / 2 完全字符串 / 3 字符串优先) |
| `source_path` | launch 中自动设为 `tts` 包 share 目录 | SDK 资源目录 |
| `tts_text` | launch 中默认 `"你好小微"` | 待合成的一段文本 |

## 启动文件

### `launch/tts_make.launch.py`

核心内容:
```python
Node(
    package="tts",
    executable="tts_node",
    parameters=[{"source_path": tts_dir},
                {"tts_text": "你好小微"},
                tts_config]
)
```

- 自动把 `get_package_share_directory('tts')` 作为 `source_path` 传入(SDK 需要)。
- 自动加载 `config/tts_params.yaml`。
- 默认合成文本写死为 `"你好小微"`,如需更改请修改 launch 或在命令行覆盖参数。

## 编译与运行

```bash
colcon build --packages-up-to tts
source install/setup.bash

# 启动 launch(推荐,自动配好 source_path 与默认参数)
ros2 launch tts tts_make.launch.py
```

## 使用示例

```bash
# 1. 用 launch 启动(合成 launch 中默认的 "你好小微")
ros2 launch tts tts_make.launch.py

# 2. 直接运行可执行,并通过命令行覆盖文本与 APPID
ros2 run tts tts_node --ros-args \
    -p appid:=<YOUR_APPID> \
    -p voice_name:=xiaoyan \
    -p tts_text:="你好，我是 WheelTec 机器人" \
    -p source_path:=$(ros2 pkg prefix tts)/share/tts

# 3. 想换合成文本时,修改 launch/tts_make.launch.py 里的
#    tts_text = {"tts_text": "..."}  或在命令行用 -p tts_text:= 覆盖
```

## 注意事项

1. **包名是 `tts`,不是 `tts_make_ros2`**。所有 `colcon build --packages-select`、`ros2 run`、`ros2 launch` 都要写 `tts`。
2. 必须在讯飞开放平台申请 `APPID`,默认 yaml 里的 `b22421bd` 是示例,过期或额度耗尽后需替换。
3. SDK 区分 x86_64 / aarch64 版本,务必使用与本机架构匹配的 `libs/`。
4. 离线 TTS 需要单独购买授权,本包默认在线模式,需要联网。
5. `source_path` 必须指向 SDK 资源所在目录(launch 已自动处理,手动 `ros2 run` 时需自己传)。
6. 合成完成后会调用系统播放器播放 WAV,需确保 ALSA / PulseAudio 设备可用。
