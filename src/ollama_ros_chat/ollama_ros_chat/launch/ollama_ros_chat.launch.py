from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    
    return LaunchDescription([
        # 服务模式节点 (同步返回)
        Node(
            package='ollama_ros_chat',
            executable='chat_service',
            name='chat_service',
            output='screen'
        ),
        # 话题模式节点 (流式输出)
        Node(
            package='ollama_ros_chat',
            executable='topic_server',
            name='ollama_topic_server',
            output='screen'
        )
    ])
