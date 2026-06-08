#!/usr/bin/env python3
"""VLA 自主导航节点.

把语音/文字指令(voice_words 或 /vla/instruction) 连同前方摄像头画面(/image_raw)
交给本地 Ollama 多模态大模型推理, 得到导航决策后:
  - goto_waypoint : 查命名航点 -> 发布 Nav2 目标点
  - relative_move : 用 TF 把"相对画面/机体"的目标换算到 map -> 发布目标点
  - stop / speak  : 停止或仅语音反馈
所有结果都会通过 TTS 话题(tts_text)语音播报。
"""

import base64
import math
import os

import cv2

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, String

from nav2_msgs.action import NavigateToPose

import tf2_geometry_msgs  # noqa: F401  注册 PoseStamped 的 TF 转换
from tf2_ros import Buffer, TransformException, TransformListener

from vla_navigation.vlm_client import VLMClient
from vla_navigation.waypoints import WaypointMap


# 麦克风唤醒时驱动节点会发布这句, 不应被当作导航指令
WAKEUP_PHRASE = '小车唤醒'


class VlaNavigator(Node):

    def __init__(self):
        super().__init__('vla_navigator')

        # ---------------- 参数 ----------------
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('vlm_model', 'qwen2.5vl:3b')
        self.declare_parameter('vlm_timeout', 60.0)
        self.declare_parameter('vlm_temperature', 0.2)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('voice_topic', 'voice_words')
        self.declare_parameter('instruction_topic', '/vla/instruction')
        self.declare_parameter('awake_topic', 'awake_flag')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('tts_topic', 'tts_text')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('use_action', False)
        self.declare_parameter('send_image', True)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('max_relative_distance', 2.0)
        self.declare_parameter('debounce_sec', 3.0)
        self.declare_parameter('waypoints_file', '')

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.base_frame = gp('base_frame').value
        self.use_action = bool(gp('use_action').value)
        self.send_image = bool(gp('send_image').value)
        self.jpeg_quality = int(gp('jpeg_quality').value)
        self.max_rel_dist = float(gp('max_relative_distance').value)
        self.debounce_sec = float(gp('debounce_sec').value)

        # ---------------- 大模型 / 航点 ----------------
        self.vlm = VLMClient(
            base_url=gp('ollama_url').value,
            model=gp('vlm_model').value,
            timeout=gp('vlm_timeout').value,
            temperature=gp('vlm_temperature').value,
            logger=self.get_logger(),
        )
        self.waypoint_map = self._load_waypoints(gp('waypoints_file').value)
        self.get_logger().info('已加载 %d 个命名航点: %s' % (
            len(self.waypoint_map.waypoints), ', '.join(self.waypoint_map.names()) or '无'))

        # ---------------- ROS 接口 ----------------
        self.bridge = CvBridge()
        self.latest_image = None

        self.goal_pub = self.create_publisher(PoseStamped, gp('goal_topic').value, 10)
        self.tts_pub = self.create_publisher(String, gp('tts_topic').value, 10)

        self.create_subscription(Image, gp('image_topic').value,
                                 self._image_cb, qos_profile_sensor_data)
        self.create_subscription(String, gp('voice_topic').value,
                                 self._instruction_cb, 10)
        self.create_subscription(String, gp('instruction_topic').value,
                                 self._instruction_cb, 10)
        self.create_subscription(Int8, gp('awake_topic').value,
                                 self._awake_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_action = None
        self._goal_handle = None
        if self.use_action:
            self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._last_instruction = ''
        self._last_stamp = self.get_clock().now()

        self.get_logger().info(
            'VLA 导航节点就绪: 模型=%s, 图像=%s, 目标发布=%s' % (
                gp('vlm_model').value, gp('image_topic').value,
                'navigate_to_pose(action)' if self.use_action else gp('goal_topic').value))

    # ----------------------------------------------------------------- helpers
    def _load_waypoints(self, configured_path):
        path = configured_path
        if not path:
            try:
                share = get_package_share_directory('vla_navigation')
                path = os.path.join(share, 'config', 'waypoints.yaml')
            except Exception:  # noqa: BLE001  找不到 share 目录时退化为空航点
                path = ''
        return WaypointMap.from_yaml(path)

    def _say(self, text):
        if not text:
            return
        self.tts_pub.publish(String(data=str(text)))
        self.get_logger().info('TTS: %s' % text)

    def _image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001  图像转换失败不应让节点崩溃
            self.get_logger().warn('图像转换失败: %s' % exc)

    def _encode_image(self):
        if not self.send_image or self.latest_image is None:
            return None
        ok, buf = cv2.imencode('.jpg', self.latest_image,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode('ascii')

    def _awake_cb(self, msg):
        if msg.data == 1:
            self._say('我在，请说指令')

    # --------------------------------------------------------------- core flow
    def _instruction_cb(self, msg):
        text = (msg.data or '').strip()
        if not text or text == WAKEUP_PHRASE:
            return
        # 去抖: 短时间内的重复指令忽略
        now = self.get_clock().now()
        if text == self._last_instruction and \
                (now - self._last_stamp) < Duration(seconds=self.debounce_sec):
            return
        self._last_instruction = text
        self._last_stamp = now

        self.get_logger().info('收到指令: %s' % text)
        image_b64 = self._encode_image()
        if self.send_image and image_b64 is None:
            self.get_logger().warn('暂无可用摄像头图像, 本次按纯文本推理')

        decision = self.vlm.query(
            instruction=text,
            waypoint_text=self.waypoint_map.describe(),
            image_b64=image_b64,
        )
        self.get_logger().info('大模型决策: %s' % decision)
        self._dispatch(decision)

    def _dispatch(self, decision):
        if not isinstance(decision, dict):
            self._say('抱歉，我没理解这条指令')
            return

        action = str(decision.get('action', 'unknown')).lower()
        say = decision.get('speak')

        if action == 'goto_waypoint':
            self._handle_goto(decision.get('waypoint'), say)
        elif action == 'relative_move':
            self._handle_relative(decision, say)
        elif action == 'stop':
            self._say(say or '好的，停下')
            self._cancel_nav()
        elif action == 'speak':
            self._say(say or '好的')
        else:  # unknown 或非法值
            self._say(say or '抱歉，我不太明白')

    def _handle_goto(self, name, say):
        wp = self.waypoint_map.match(name)
        if wp is None:
            self.get_logger().warn('未知航点: %r' % name)
            self._say('我不知道%s在哪里' % (name or '那个地点'))
            return
        z, w = wp.quaternion()
        pose = self._make_pose(self.map_frame, wp.x, wp.y, z, w)
        self._say(say or ('好的，正在前往%s' % wp.name))
        self._send_goal(pose)
        self.get_logger().info('前往航点 %s -> (x=%.2f, y=%.2f)' % (wp.name, wp.x, wp.y))

    def _handle_relative(self, decision, say):
        try:
            distance = float(decision.get('distance', 0.0) or 0.0)
            angle_deg = float(decision.get('angle', 0.0) or 0.0)
        except (TypeError, ValueError):
            self._say('这个相对目标我算不出来')
            return

        distance = max(-self.max_rel_dist, min(self.max_rel_dist, distance))
        yaw = math.radians(angle_deg)

        # 机体坐标系下的目标点: 先转 yaw, 再沿前进方向走 distance
        local = self._make_pose(
            self.base_frame,
            distance * math.cos(yaw),
            distance * math.sin(yaw),
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )
        local.header.stamp = Time().to_msg()  # stamp=0 表示用最新可用 TF

        try:
            goal = self.tf_buffer.transform(
                local, self.map_frame, timeout=Duration(seconds=0.5))
        except TransformException as exc:
            self.get_logger().warn('TF 转换失败 (%s -> %s): %s' % (
                self.base_frame, self.map_frame, exc))
            self._say('我暂时定位不到自己的位置')
            return

        self._say(say or '好的，马上过去')
        self._send_goal(goal)
        self.get_logger().info('相对移动: 距离=%.2fm, 角度=%.1f° -> map(x=%.2f, y=%.2f)' % (
            distance, angle_deg, goal.pose.position.x, goal.pose.position.y))

    # --------------------------------------------------------------- nav output
    def _make_pose(self, frame, x, y, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def _send_goal(self, pose):
        if self.use_action and self.nav_action is not None:
            if not self.nav_action.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn('navigate_to_pose 动作服务器不可用, 改用话题发布')
                self.goal_pub.publish(pose)
                return
            goal = NavigateToPose.Goal()
            goal.pose = pose
            future = self.nav_action.send_goal_async(goal)
            future.add_done_callback(self._goal_response_cb)
        else:
            self.goal_pub.publish(pose)

    def _goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001  动作请求异常时不崩溃
            self.get_logger().warn('发送导航目标失败: %s' % exc)
            return
        if handle is not None and handle.accepted:
            self._goal_handle = handle

    def _cancel_nav(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self.get_logger().info('收到停止指令')


def main(args=None):
    rclpy.init(args=args)
    node = VlaNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
