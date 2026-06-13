#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
from typing import List, Dict, Optional
import time

class OllamaChatNode(Node):
    def __init__(self):
        super().__init__('ollama_topic_server')
        
        # 创建发布者和订阅者
        self.response_publisher = self.create_publisher(
            String, 
            'chat_response', 
            10
        )
        self.message_subscription = self.create_subscription(
            String,
            'chat_message',
            self.message_callback,
            10
        )
        
        # Ollama配置
        self.base_url = "http://localhost:11434"
        self.use_model = "qwen3-vl:2b"
        self.stream = True
        self.temperature = 0.5
        self.history_length = 10
        self.available_models = []
        
        # 初始化模型
        self.initialize_models()
        self.select_model()
        
        self.conversation_history = [{"role": "system", "content": f"You are {self.use_model}, a helpful assistant developed by Alibaba Cloud."}]
        
        self.get_logger().info('Ollama Chat Topic Server Node initialized')

    def initialize_models(self):
        """Query available Ollama models"""
        try:
            response = requests.get(f"{self.base_url}/api/tags")
            if response.status_code == 200:
                models = response.json()['models']
                self.available_models = [model['name'] for model in models]
                self.get_logger().info(f"Available models: {', '.join(self.available_models)}")
                return self.available_models
            else:
                self.get_logger().error(f"Failed to get models. Status code: {response.status_code}")
                return []
        except Exception as e:
            self.get_logger().error(f"Error getting models: {e}")
            return []

    def select_model(self) -> None:
        """Select first available model"""
        target_model = "qwen3-vl:2b"
        if target_model in self.available_models:
            self.use_model = target_model
        elif self.available_models:
            self.use_model = self.available_models[0]
        
        if self.use_model:
            self.get_logger().info(f"Selected model: {self.use_model}")
        else:
            self.get_logger().error("No models available")

    def message_callback(self, msg):
        """Handle incoming chat messages"""
        try:
            # 解析接收到的消息
            message_data = json.loads(msg.data)
            user_message = message_data.get('content', '')
            images = message_data.get('images', [])
            
            # 更新对话历史
            msg_dict = {"role": "user", "content": user_message}
            if images:
                msg_dict["images"] = images
            
            self.conversation_history.append(msg_dict)
            print("Received message:", user_message)
            if images:
                print(f"Received {len(images)} images")

            # 获取响应
            time_start = time.time()
            response_content = self.get_response(self.conversation_history)
            time_end = time.time()
            print("Response_content:", response_content)
            print("Time taken:", time_end - time_start)
            
            if response_content:
                # 更新对话历史
                self.conversation_history.append({"role": "assistant", "content": response_content})
                self.conversation_history = self.process_data(self.conversation_history)
                
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def get_response(self, messages: List[Dict[str, str]]) -> Optional[str]:
        """Get response from Ollama model using /api/chat"""
        try:
            url = f"{self.base_url}/api/chat"
            data = {
                "model": self.use_model,
                "messages": messages,
                "stream": self.stream,
                "options": {
                    "temperature": self.temperature
                }
            }

            response = requests.post(url, json=data, stream=self.stream)
            if response.status_code == 200:
                full_response = ""
                for line in response.iter_lines():
                    if line:
                        json_response = json.loads(line)
                        if 'message' in json_response:
                            chunk = json_response['message'].get('content', '')
                            full_response += chunk
                            # 发布部分响应
                            publish_msg = String()
                            publish_msg.data = json.dumps({
                                "content": chunk,
                                "model": self.use_model,
                                "is_done": json_response['done']
                            })
                            self.response_publisher.publish(publish_msg)
                        if json_response.get('done', False):
                            return full_response
            else:
                self.get_logger().error(f"Error: Received status code {response.status_code}")
                return None

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            return None

    def process_data(self, data_list: List[Dict[str, str]]) -> List[Dict[str, str]]:
        """Maintain conversation history within specified length"""
        if self.history_length <= 0:
            raise ValueError("History length must be a positive integer")
        return data_list[-self.history_length:]

def main(args=None):
    rclpy.init(args=args)
    chat_server = OllamaChatNode()
    try:
        rclpy.spin(chat_server)
    except KeyboardInterrupt:
        pass
    finally:
        chat_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
