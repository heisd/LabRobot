# VLM 自然语言抓取使用说明

`vlm_grab_node.py` 让机械臂"**看画面 + 理解一句自然语言 → 选出该抓的物体 → 抓取**"。
你在 Dashboard 里发一句话（可以是间接的，比如"我渴了"），节点把当前相机画面 + 这句话
交给视觉语言模型（VLM），让它框出该抓的物体，再走统一接口抓取。

## 一、流程

```
Dashboard 输入指令 ──/vlm/instruction──> vlm_grab_node
                                          │ 取当前彩色帧 + 指令
                                          ▼
                                   VLM (OpenAI兼容 / Anthropic)
                                          │ 返回 {found,label,reason,bbox}
                                          ▼
              框中心 + 深度 → 针孔反投影 → 持续广播 target_frame + /grab_target/distance
                                          │
                       ├─ /vlm/result(String) → Dashboard 显示"理解为…"
                       └─ auto_grab=true → 调用 /obj_grab_service 抓取
```

接口与 HSV/YOLO/KCF 一致（同样的相机话题、`target_frame`、`/grab_target/distance`），
所以 `grab_service_node` 无需改动。

## 二、VLM 接口（本地或云端，二选一）

用 `provider` 参数切换，**只用 Python 标准库，无额外 pip 依赖**，API Key 从环境变量读取：

| provider | 说明 | 配置 |
|----------|------|------|
| `openai`（默认） | OpenAI 兼容 `/chat/completions`。**可指向本地或云端** | `api_base`, `model`, 环境变量 `OPENAI_API_KEY` |
| `anthropic` | Anthropic Claude `/v1/messages` | `api_base=https://api.anthropic.com`, `model=claude-…`, 环境变量 `ANTHROPIC_API_KEY` |

**本地部署（推荐，隐私 + 不依赖外网）**：在 Jetson 或局域网用 Ollama / vLLM / llama.cpp
跑一个支持视觉的模型（如 `qwen2-vl`、`llava`），它们都提供 OpenAI 兼容接口：

```bash
# 例: Ollama
ollama run qwen2-vl
# 然后让节点指向它(无需 key):
ros2 run grab_demo vlm_grab_node.py --ros-args \
  -p provider:=openai \
  -p api_base:=http://localhost:11434/v1 \
  -p model:=qwen2-vl
```

**云端 OpenAI**：

```bash
export OPENAI_API_KEY=sk-xxx
ros2 run grab_demo vlm_grab_node.py --ros-args -p model:=gpt-4o-mini
```

**云端 Anthropic**：

```bash
export ANTHROPIC_API_KEY=sk-ant-xxx
ros2 run grab_demo vlm_grab_node.py --ros-args \
  -p provider:=anthropic -p model:=claude-3-5-sonnet-20241022
```

## 三、运行

整套（相机 + 机械臂 + VLM + 抓取服务）：

```bash
ros2 launch grab_demo vlm_grab.launch.py
```

> 启动前记得 `export` 好对应的 API Key（除非用本地无 key 的服务）。

## 四、在 Dashboard 里用

1. "功能启动"页启动 **VLM 语言抓取**；
2. 回到"监控与控制"页，在 **VLM 自然语言抓取** 卡片输入一句话，回车或点"发送指令"；
3. 卡片会显示节点的理解结果，例如 `✅ 理解为 [红色的瓶子] (用户想喝水), 距离 0.42m`；
4. 若 `auto_grab=true`（默认），机械臂随即调用抓取服务去抓。

也可命令行直接发指令：

```bash
ros2 topic pub --once /vlm/instruction std_msgs/String "{data: '把香蕉拿给我'}"
ros2 topic echo /vlm/result          # 看理解结果
ros2 run rqt_image_view rqt_image_view /vlm_node/vlm_image   # 看框选可视化
```

## 五、参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `provider` | `openai` | `openai`（兼容本地/云）或 `anthropic` |
| `api_base` | `https://api.openai.com/v1` | 接口地址；本地填 `http://<ip>:<port>/v1` |
| `model` | `gpt-4o-mini` | 模型名 |
| `api_key_env` | `""` | 自定义 Key 的环境变量名；留空按 provider 取默认 |
| `request_timeout` | `30` | VLM 请求超时(秒) |
| `auto_grab` | `true` | 理解到目标后是否自动调用抓取服务 |
| `grab_service` | `/obj_grab_service` | 抓取服务名 |
| `rgb_topic`/`depth_topic`/`camera_info_topic` | 同 HSV/YOLO/KCF | 相机话题 |
| `camera_frame`/`target_frame`/`z_offset` | 同其它 | 坐标系与 Z 补偿 |
| `instruction_topic`/`result_topic` | `/vlm/instruction` / `/vlm/result` | 指令与结果话题 |
| `publish_debug_image` | `true` | 发布框选可视化到 `~/vlm_image` |

## 六、说明与注意

- VLM 调用在独立线程里进行（网络阻塞不卡 ROS）；同一时刻只处理一条指令，处理中再发会提示稍候。
- 目标点确定后，节点以 15Hz **持续广播** `target_frame`，确保抓取服务能稳定查到 TF。
- VLM 给的像素框不一定精确；本节点对归一化坐标、`point` 点位都做了兼容解析。
- `auto_grab=true` 会让机械臂真实运动，请在安全环境下使用；不想自动抓可设 `auto_grab:=false`，
  只发布 `target_frame`，再用别的方式触发抓取。
- 需要联网（云端）或本地 VLM 服务可达；网络/服务异常时结果会回 `❌ VLM 调用失败`。
