# ollama_ros_chat (基于 Ollama 大语言模型的 ROS2 对话节点)

## 概述

`ollama_ros_chat` 是一个 ROS2 (ament_python) 功能包，封装了本地部署的 Ollama 大语言模型推理后端，并通过 ROS2 的两种通信范式（话题 Topic 与服务 Service）对外提供对话能力。

该包属于轮趣科技 (Wheeltec) S300 机器人系列软件栈，是机器人语音对话/智能交互的核心 LLM 推理桥接层。它通过 HTTP 直接访问运行在本机 `http://localhost:11434` 上的 Ollama 服务，并将其能力以 ROS2 接口的形式暴露给上层节点（例如 TTS、ASR、语音控制、智能问答等模块）。

该包同时提供四个可执行节点：服务端 + 客户端各一对，使用者可根据具体场景选择基于 Service 的同步调用模式，或基于 Topic 的异步流式订阅模式。

## 目录结构

```
ollama_ros_chat/
├── launch/
│   └── ollama_ros_chat.launch.py     # 启动 chat_service 节点
├── ollama_ros_chat/                  # Python 源码目录
│   ├── __init__.py
│   ├── ollama_service.py             # 基于 ROS2 Service 的服务端
│   ├── ollama_client.py              # 基于 ROS2 Service 的客户端 (交互式终端)
│   ├── ollama_topic_server.py        # 基于 ROS2 Topic 的服务端
│   └── ollama_topic_client.py        # 基于 ROS2 Topic 的客户端 (交互式终端)
├── resource/
│   └── ollama_ros_chat
├── test/                             # 默认的 ament_python 测试
├── package.xml
├── setup.py
└── setup.cfg
```

## 依赖项

`package.xml` 中声明如下：

- buildtool：`ament_python`（通过 `<export><build_type>` 指定）
- depend (build/exec)：
  - `rclpy`
  - `std_msgs`
  - `ollama_ros_msgs`（同仓库下的接口包，提供 `Chat.srv`）
- test_depend：`ament_copyright`、`ament_flake8`、`ament_pep257`、`python3-pytest`

额外的运行时 Python 依赖（在源码 import 中使用，需要预先安装）：

- `requests`（与 Ollama HTTP API 通信）
- 本机已运行 Ollama 服务（监听 `http://localhost:11434`）

## 节点说明

### 1. chat_service (`ollama_service.py`)

- 节点名：`ollama_server`
- 作用：作为 ROS2 服务端，提供同步的 LLM 对话。
- 提供的服务：
  - `/chat_service`，类型 `ollama_ros_msgs/srv/Chat`
    - 请求：`string content`（用户输入文本）
    - 应答：`string content`（LLM 回答）、`string model`（实际使用的模型名）、`bool is_done`
- 内部行为：
  - 启动时调用 `GET http://localhost:11434/api/tags` 拉取本机已有的模型列表
  - 自动选择列表中的第一个模型 (`self.available_models[0]`)
  - 启动时调用 `POST /api/generate` 对所选模型做一次预热
  - 默认参数：`stream=False`、`temperature=0.5`、`history_length=10`
  - 维护一个最多 10 条记录的对话历史 (system / user / assistant 三种角色)
- 暂未通过 ROS2 参数声明对外暴露上述配置（如需修改 URL/温度/历史长度，需要直接修改源码）。

### 2. chat_client (`ollama_client.py`)

- 节点名：`ollama_client`
- 作用：交互式命令行客户端，调用 `chat_service`。
- 调用的服务：`/chat_service`（`ollama_ros_msgs/srv/Chat`）
- 行为：循环读取用户在终端中的输入并通过异步服务请求发送，收到响应后打印；输入 `exit` 退出。

### 3. topic_server (`ollama_topic_server.py`)

- 节点名：`ollama_topic_server`
- 订阅话题：
  - `/chat_message`，类型 `std_msgs/String`
    - 数据为 JSON 字符串：`{"content": "用户消息"}`
- 发布话题：
  - `/chat_response`，类型 `std_msgs/String`
    - 数据为 JSON 字符串：`{"content": "<chunk>", "model": "<模型名>", "is_done": <bool>}`
    - 按 Ollama 流式输出逐 chunk 发布（注意：源码中虽然向 Ollama 请求时使用 `stream=False`，但仍会按 `iter_lines` 拆分并按 chunk 发布）
- 其它配置同 `chat_service`：默认 URL `http://localhost:11434`、`temperature=0.5`、`history_length=10`，自动选择第一个可用模型。

### 4. topic_client (`ollama_topic_client.py`)

- 节点名：`ollama_topic_client`
- 发布话题：`/chat_message`（`std_msgs/String`，JSON 包裹 `content` 字段）
- 订阅话题：`/chat_response`（`std_msgs/String`），收到一条响应后按 chunk 打印到终端
- 行为：使用一个独立线程 spin，主线程循环 `input()`；当上一次响应未结束（`is_done=False`）时不会接受下一次输入。

## 启动文件

### launch/ollama_ros_chat.launch.py

仅启动一个节点：

| 包名 | 可执行文件 | 节点名 | 输出 |
|------|-----------|--------|------|
| `ollama_ros_chat` | `chat_service` | `chat_service` | screen |

该 launch 文件没有声明任何参数，纯粹用于拉起服务端。客户端需要使用 `ros2 run` 单独启动。

## 消息/服务/动作定义

本包本身不包含接口定义。所使用的 `Chat.srv` 定义在同仓库的 `ollama_ros_msgs` 包中，字段如下：

```
# Request
string content
---
# Response
string content
string model
bool is_done
```

## 参数配置

本包当前未提供 yaml 配置文件，也未通过 `declare_parameter` 暴露 ROS2 参数。下列“配置项”是硬编码在 Python 源码中的，如需修改请编辑 `ollama_service.py` / `ollama_topic_server.py`：

| 名称 | 含义 | 默认值 |
|------|------|--------|
| `base_url` | Ollama HTTP 服务地址 | `http://localhost:11434` |
| `stream` | 是否使用流式请求 | `False` |
| `temperature` | 采样温度 | `0.5` |
| `history_length` | 保留的最大对话轮数 | `10` |
| 默认 system prompt | 系统提示词 | `"You are a helpful assistant"` |
| `use_model` | 使用的模型 | 启动时通过 `/api/tags` 拉取后，自动选用列表第一个 |

## 编译与运行

在 ROS2 (推荐 Humble) 工作空间根目录下：

```bash
cd ~/wheeltec_S300
colcon build --packages-select ollama_ros_msgs ollama_ros_chat
source install/setup.bash
```

先确保本机已安装并启动 Ollama，且至少拉取了一个模型，例如：

```bash
# 安装 Ollama 后
ollama pull qwen2:1.5b   # 或任意其他模型
ollama serve             # 通常已经作为后台服务启动
```

启动服务端：

```bash
ros2 launch ollama_ros_chat ollama_ros_chat.launch.py
# 或：
ros2 run ollama_ros_chat chat_service
```

启动交互式客户端：

```bash
# Service 版本
ros2 run ollama_ros_chat chat_client

# Topic 版本
ros2 run ollama_ros_chat topic_server     # 另一种服务端实现
ros2 run ollama_ros_chat topic_client
```

也可直接通过命令行调用服务：

```bash
ros2 service call /chat_service ollama_ros_msgs/srv/Chat "{content: '你好'}"
```

## 使用示例

典型流程（Service 版本）：

1. 启动 Ollama：`ollama serve`
2. 启动服务端：`ros2 run ollama_ros_chat chat_service`
3. 在另一个终端启动客户端：`ros2 run ollama_ros_chat chat_client`
4. 在客户端中输入问题，例如 `user: 介绍一下你自己`，服务端会查询 Ollama 并将回答返回打印。

典型流程（Topic 版本）：

1. 启动 Ollama
2. `ros2 run ollama_ros_chat topic_server`
3. `ros2 run ollama_ros_chat topic_client`
4. 在客户端输入文字，会以 JSON 形式发布到 `/chat_message`；服务端按 chunk 把回答发布到 `/chat_response`，客户端逐字打印。

也可以让其它节点通过订阅 `/chat_response` / 发布 `/chat_message` 直接对接到自己的语音 / 文本流水线中（例如把 ASR 结果作为 `content` 发布出去，把 LLM 回复转给 TTS 合成）。

## 注意事项

1. 本包依赖本机 Ollama 服务，请确认 `http://localhost:11434` 可访问、且至少存在一个已下载的模型，否则节点会打印 `No models available` 并无法处理请求。
2. 自动选择 `available_models[0]` 作为推理模型，启动顺序或模型列表变化都会影响实际使用的模型；如需固定模型，应修改 `select_model()` 源码。
3. Service 版的请求是同步阻塞的，长文本回答会阻塞该次请求；默认 HTTP 超时为 120s。
4. 对话历史长度上限为 10 条（含 system prompt），超出会丢弃最早的内容；若需要长记忆需调整 `history_length` 并在 `process_data` 中保留 system 消息。
5. 接口包 `ollama_ros_msgs` 必须先于本包构建，否则会找不到 `Chat.srv`。
6. 运行需安装 Python 包 `requests`，例如 `pip install requests`。
