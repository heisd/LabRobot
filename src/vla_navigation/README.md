# vla_navigation —— 语音 + 视觉大模型(VLA)自主导航

把**语音/文字指令**和**机器人前方摄像头画面**交给机器人本机的 **Ollama 多模态大模型**推理，
模型输出一个导航决策，节点据此向 **Nav2** 发布目标点，并通过 **TTS** 语音反馈。

> 这里的 “VLA” 是面向**轮式底盘导航**的 *Vision-Language-Action* 落地形态：
> **动作(Action) = 一个导航目标(Nav2 goal)**，而不是机械臂的低层关节动作。
> 它复用了本仓库已有的麦克风、摄像头、Ollama 桥接与 Nav2，不需要额外的大显卡训练。

## 数据流

```
[麦克风 wheeltec_mic] --voice_words(String)--> ┐
[手动测试 /vla/instruction(String)] ----------> ├─> [vla_navigator] --query--> [Ollama VLM]
[摄像头 /image_raw(Image)] -------------------> ┘                                   │
                                                                                   ▼
                                                                     决策 JSON {action, ...}
                                                                                   │
                 ┌─────────────────────────────────┬────────────────────┬─────────┘
                 ▼                                  ▼                    ▼
        goto_waypoint                       relative_move            speak / stop
   查 waypoints.yaml -> goal_pose      TF: base_footprint->map        发 tts_text
                 └──────────── /goal_pose(PoseStamped, map) ──────────> [Nav2]
                                    所有结果 --tts_text(String)--> [tts 节点语音播报]
```

## 大模型决策格式

节点要求模型严格返回如下 JSON（`vlm_client.py` 中 `format=json` 强制）：

```json
{
  "action": "goto_waypoint | relative_move | stop | speak | unknown",
  "waypoint": "目标航点名称(action=goto_waypoint 时)",
  "distance": 1.0,          // 米, 相对前进距离(action=relative_move 时)
  "angle": -30.0,           // 度, 需要转过的角度(正=左转)
  "speak": "好的，正在前往厨房"   // 一句中文反馈, 总会播报
}
```

- `goto_waypoint`：去 `config/waypoints.yaml` 里**预设的命名航点**（如“去厨房 / 去充电桩 / 去 I 点”）。
- `relative_move`：根据**当前画面/机体**做相对移动（如“向前一点 / 左转去那扇门”），用 TF 换算到 `map`。
- `stop` / `speak` / `unknown`：停止、纯语音回答、无法理解。

## 依赖与准备（在机器人/实车上）

1. ROS 2 Humble + 本仓库已编译（mic、usb_cam、Nav2 等）。
2. 安装并启动 **Ollama**，拉取一个多模态模型（默认 `qwen2.5vl:3b`，中文友好；也可用 `llava`）：
   ```bash
   ollama pull qwen2.5vl:3b
   # 确认服务在 http://localhost:11434
   ```
3. Python 依赖：`requests`、`PyYAML`、`opencv`(随 cv_bridge)、`cv_bridge`、`tf2_geometry_msgs`。

## 编译

```bash
cd ~/wheeltec_S300
colcon build --packages-select tts vla_navigation
source install/setup.bash
```

## 运行

分别在不同终端启动（顺序无所谓，建议先起 Nav2 与摄像头）：

```bash
# 1) 底盘 + Nav2(已建好地图 WHEELTEC.yaml)
ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py

# 2) 摄像头
ros2 launch usb_cam usb_cam_launch.py        # 发布 /image_raw

# 3) 语音输入(麦克风离线识别) + TTS 播报
ros2 launch wheeltec_mic_ros2 base.launch.py # 发布 voice_words / awake_flag
ros2 launch tts tts_make.launch.py           # 订阅 tts_text, 用 aplay 播放

# 4) VLA 导航大脑
ros2 launch vla_navigation vla_navigation.launch.py
```

不接麦克风也能测试——直接发文字指令：

```bash
ros2 topic pub --once /vla/instruction std_msgs/msg/String "{data: '去厨房'}"
ros2 topic pub --once /vla/instruction std_msgs/msg/String "{data: '向前走一米然后左转'}"
```

让机器人开口说话（验证语音模块改造）：

```bash
ros2 topic pub --once /tts_text std_msgs/msg/String "{data: '你好，我是小车'}"
```

## 配置

- `config/vla_params.yaml`：Ollama 地址/模型、话题名、坐标系、`use_action`、相对移动上限等。
- `config/waypoints.yaml`：命名航点（**坐标需按你自己建好的地图实测标定**，示例中的 I/J/K
  沿用 `wheeltec_mic` 里 `command_recognition` 的默认航点）。

### 重要参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `vlm_model` | `qwen2.5vl:3b` | Ollama 已 pull 的多模态模型名 |
| `image_topic` | `/image_raw` | usb_cam；Astra 改 `/camera/color/image_raw` |
| `send_image` | `true` | 是否把画面喂给模型；纯航点导航可设 false 提速 |
| `use_action` | `false` | false=发 `goal_pose`；true=用 `navigate_to_pose` action（可取消） |
| `base_frame` | `base_footprint` | 相对移动用的机体坐标系 |
| `waypoints_file` | 空 | 空则用本包内 `config/waypoints.yaml` |

## 与现有语音模块的关系

- 输入沿用 `wheeltec_mic` 的 `voice_words`（离线命令词）。已在
  `wheeltec_mic_ros2/config/call.bnf` 的 `<navigation>` 中扩充了命名目的地
  （原点/厨房/客厅/卧室/餐厅/书房/门口/充电桩/前台/会议室…）以及
  “去X / 导航到X / 前往X / 到X”等说法，并保留原有 `去I/J/K点`。
  该语法在语音节点启动时会自动重建生效（如未生效，删除
  `config/msc/res/asr/GrmBuilld` 缓存后重启）。若要**自由说话**，仍需接入在线 ASR（本期未做）。
- 输出复用改造后的 `tts` 节点：它现在**订阅 `tts_text` 话题**并用 `aplay` 真正播放，
  而不再是开机只合成一次 WAV。

## 并发模型

大模型推理可能耗时数秒到数十秒。为避免阻塞 ROS 执行器（推理期间收不到图像/TF/语音），
`vla_navigator` 把推理放在**独立工作线程**：指令回调只负责抓取当前帧并入队，
工作线程串行取出、调用 Ollama 并分发动作。队列已满（上一条仍在推理）时新指令会被忽略并告警。

## 局限

- 在本仓库的云端开发容器里**无法编译/运行**（无 ROS/GPU/Ollama），所有代码需在实车上构建验证。
- 离线命令词语法限制了能说的话；自由自然语言指令需后续接在线 ASR。
- `relative_move` 依赖 `map<-base_footprint` 的 TF（即已定位）；未定位时会语音提示。
