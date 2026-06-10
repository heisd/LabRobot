# 语音指令使用说明 (本地 Whisper → VLM)

`voice_instruction_node.py` 给 VLM 抓取加了一张"嘴"：**说话 → 本地语音识别(Whisper) → 文字发到
`/vlm/instruction`**。VLM 节点(`vlm_grab_node`)**无需任何改动**，照常用这句话去理解并抓取。

```
麦克风 ──(能量VAD分句)──> 本地 Whisper 转写 ──文字──> /vlm/instruction ──> VLM 理解 + 闭环抓取
                                                  └──> /voice/text (Dashboard 显示)
```

## 一、为什么是"本地"

- **离线、隐私**：识别在设备(Jetson/PC)本地完成，不依赖外网；
- 用 **faster-whisper**(默认，CPU/GPU 都快) 或 **openai-whisper**；
- 依赖缺失会**优雅降级**：没装库时节点不崩，发布 `/voice/status=error` 并打印安装提示。

## 二、安装依赖

```bash
# 录音 + 识别(推荐)
pip install sounddevice faster-whisper
# 或者用 openai 版: pip install sounddevice openai-whisper

# 系统音频库(Ubuntu/Jetson)
sudo apt install -y libportaudio2     # sounddevice 需要 PortAudio
```

> 确认有麦克风且有权限：`python3 -c "import sounddevice as sd; print(sd.query_devices())"`
> 列出设备，必要时用参数 `mic_device:=<索引>` 指定。

## 三、运行

随 VLM 一起启动（默认关闭，用 `use_voice:=true` 打开）：

```bash
ros2 launch grab_demo vlm_grab.launch.py use_voice:=true
```

或单独跑语音节点（配合已在跑的 VLM）：

```bash
ros2 run grab_demo voice_instruction_node.py --ros-args \
  -p backend:=faster-whisper -p model:=small -p language:=zh -p device:=cpu
```

> 中文识别建议 `model:=small` 或 `medium`（`base/tiny` 更快但中文易错）。有 GPU 用 `device:=cuda`。

## 四、两种说话方式

1. **持续聆听（默认 `enabled:=true`）**：一直监听，用能量 VAD 自动分句，检测到一句话就识别并发出。
2. **按一下说一句**：调用服务录一句（适合嘈杂环境/避免误触发）：
   ```bash
   ros2 service call /voice_node/listen_once std_srvs/srv/Trigger
   ```
   或在 Dashboard 点【🎤 说一句】。

随时开/关持续聆听：
```bash
ros2 topic pub --once /voice/enable std_msgs/Bool "{data: false}"   # 关
```

## 五、在 Dashboard 里用

监控页新增 **【语音指令】** 卡片：
- 显示状态：`idle / listening / transcribing / error…`；
- 【🎤 说一句】：录一句话（listen_once）；
- 【开始持续聆听】/【停止聆听】：切换 `enabled`；
- 识别出的文字实时显示，并自动发给 VLM（卡片下方 VLM 结果会跟着更新）。

> 需用 `use_voice:=true` 启动 VLM 任务，节点名为 `voice_node`（服务 `/voice_node/listen_once`）。

## 六、参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `instruction_topic` | `/vlm/instruction` | 识别文字发到这里(VLM 订阅) |
| `text_topic` / `status_topic` | `/voice/text` / `/voice/status` | 识别文字 / 状态(Dashboard 显示) |
| `publish_to_instruction` | `true` | 是否把识别文字转成 VLM 指令 |
| `backend` | `faster-whisper` | `faster-whisper` 或 `whisper`(openai-whisper) |
| `model` | `base` | whisper 模型: tiny/base/small/medium(中文建议 small+) |
| `language` | `zh` | 识别语言(留空=自动) |
| `device` | `cpu` | `cpu` 或 `cuda` |
| `compute_type` | `int8` | faster-whisper 计算精度(int8/float16/float32) |
| `sample_rate` | `16000` | 采样率(Whisper 要 16k) |
| `mic_device` | `-1` | 麦克风设备索引, -1=默认 |
| `vad_threshold` | `0.012` | 能量 VAD 阈值(RMS), 环境吵就调大 |
| `silence_sec` | `0.8` | 句末静音多久判定结束 |
| `min_speech_sec` / `max_speech_sec` | `0.3` / `12` | 一句话最短/最长 |
| `listen_timeout` | `8.0` | 【说一句】等待开口的上限(秒) |
| `enabled` | `true` | 是否持续聆听 |

## 七、和其它能力的关系

- 识别文字走的就是 VLM 的指令入口，所以 VLM 的**安全二次确认 / 仲裁(手动接管打断) / 闭环抓取**全都生效：
  说"把瓶子递给我" → VLM 理解 → 默认等你点【确认抓取】→ 闭环抓取；手动接管会随时打断。
- 没接入 VLM 大模型时，VLM 会按 `VLM_GUIDE.md` 第六节给出"未接入大模型"的明确日志/提示。
