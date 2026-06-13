#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
import base64
import time
import re
from ollama_ros_msgs.srv import Chat

class ChatClientNode(Node):
    def __init__(self):
        super().__init__('ollama_client')
        
        # 创建服务客户端
        self.client = self.create_client(Chat, 'chat_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Chat Client Node initialized')
        self.future = None

    def send_message(self, message: str, images: list = None):
        """Send a chat message with optional images"""
        request = Chat.Request()
        request.content = message if message else "请分析这张图片"
        if images:
            request.images = images
        self.future = self.client.call_async(request)
    
    def response_callback(self):
        """Handle incoming chat responses"""
        try:
            response = self.future.result()
            content = response.content
            print(f"\nassistant [{response.model}]: {content}")
            print("response done.")
        except Exception as e:
            self.get_logger().error(f"Error processing response: {e}")

def main(args=None):
    rclpy.init(args=args)
    ollama_client = ChatClientNode()
    time.sleep(1)
    print("\nChat Client Node is running")
    print("Type 'exit' to quit.")
    print("You can enter a file path and a question in the same line.")
    
    try:
        while True:
            user_input = input("\nuser: ").strip()
            if not user_input:
                continue
            if user_input.lower() == 'exit':
                break
            
            potential_path = None
            message = user_input
            
            # 1. Try to find path in quotes
            match = re.search(r"['\"](.*?)['\"]", user_input)
            if match:
                path_in_quotes = match.group(1)
                if os.path.isfile(path_in_quotes):
                    potential_path = path_in_quotes
                    message = user_input.replace(match.group(0), "").strip()
            
            # 2. If not found, check if the whole input is a path
            if not potential_path and os.path.isfile(user_input):
                potential_path = user_input
                message = ""

            # 3. If still not found, check parts (split by space)
            if not potential_path:
                parts = user_input.split()
                for i, part in enumerate(parts):
                    if os.path.isfile(part):
                        potential_path = part
                        message = " ".join(parts[:i] + parts[i+1:]).strip()
                        break

            images_base64 = []
            if potential_path:
                try:
                    with open(potential_path, "rb") as image_file:
                        encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
                        images_base64.append(encoded_string)
                    print(f"Image '{potential_path}' loaded.")
                    if not message:
                        message = input("message for this image: ").strip()
                except Exception as e:
                    print(f"Error loading image: {e}")

            ollama_client.send_message(message, images_base64)
            rclpy.spin_until_future_complete(ollama_client, ollama_client.future)
            if ollama_client.future.done():
                ollama_client.response_callback()
                    
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        ollama_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
