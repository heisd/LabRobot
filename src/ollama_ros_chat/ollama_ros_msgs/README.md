# ollama_ros_msgs

`ollama_ros_chat` 节点使用的 ROS2 服务接口包,封装与本地大语言模型(LLM)对话的请求 / 响应字段。

## 概述

纯接口包(无可执行节点)。仅定义一个 `Chat` 服务,用于客户端把一句话送给 LLM,服务端返回 LLM 回复。

## 目录结构

```
ollama_ros_msgs/
├── CMakeLists.txt
├── package.xml
└── srv/
    └── Chat.srv
```

## 依赖项

- buildtool: `ament_cmake`、`rosidl_default_generators`
- 接口依赖: 无(仅 string、bool)
- 运行依赖: `rosidl_default_runtime`
- 接口分组: `rosidl_interface_packages`

## 服务定义

### `srv/Chat.srv`

```
# Request
string content       # 用户输入的对话内容
---
# Response
string content       # 模型回复
string model         # 实际使用的模型名(如 "qwen2.5:7b")
bool   is_done       # 是否生成结束
```

## 编译

```bash
colcon build --packages-select ollama_ros_msgs
source install/setup.bash
ros2 interface show ollama_ros_msgs/srv/Chat
```

## 使用示例

```python
from ollama_ros_msgs.srv import Chat
from rclpy.node import Node

class ChatClient(Node):
    def __init__(self):
        super().__init__('chat_client')
        self.cli = self.create_client(Chat, '/chat')

    def ask(self, text):
        req = Chat.Request()
        req.content = text
        return self.cli.call_async(req)
```

## 注意事项

1. `is_done` 一般表示模型完整返回(非流式),如使用流式生成需要业务层自行拼接。
2. 修改字段后,所有使用本接口的包都需要重新编译。
