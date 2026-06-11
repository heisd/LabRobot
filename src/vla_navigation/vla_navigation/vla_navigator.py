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
import json
import math
import os
import queue
import threading

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
from vla_navigation.waypoints import Waypoint, WaypointMap


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
        self.declare_parameter('status_topic', 'vla/status')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('use_action', False)
        self.declare_parameter('send_image', True)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('max_relative_distance', 2.0)
        self.declare_parameter('debounce_sec', 3.0)
        self.declare_parameter('waypoints_file', '')
        # 用户航点文件: dashboard 地图选点保存于此(不随 colcon build 被覆盖)。
        # 文件存在时优先于 waypoints_file/包内 config 加载 —— 一次标定永久生效。
        self.declare_parameter('user_waypoints_file', '~/.ros/vla_waypoints.yaml')
        self.declare_parameter('waypoints_topic', 'vla/waypoints')
        self.declare_parameter('waypoint_cmd_topic', 'vla/waypoint_cmd')

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
        self.user_waypoints_file = os.path.expanduser(
            str(gp('user_waypoints_file').value or '').strip())
        self.waypoint_map = self._load_waypoints(gp('waypoints_file').value)
        self.get_logger().info('已加载 %d 个命名航点: %s' % (
            len(self.waypoint_map.waypoints), ', '.join(self.waypoint_map.names()) or '无'))

        # ---------------- ROS 接口 ----------------
        self.bridge = CvBridge()
        self.latest_image = None

        self.goal_pub = self.create_publisher(PoseStamped, gp('goal_topic').value, 10)
        self.tts_pub = self.create_publisher(String, gp('tts_topic').value, 10)
        # 给 dashboard / 调试用的可读状态(指令/决策/导航)
        self.status_pub = self.create_publisher(String, gp('status_topic').value, 10)
        # 航点列表(JSON)广播: 启动/变更时发布, 并周期重发让晚连上的面板也能拿到
        self.waypoints_pub = self.create_publisher(String, gp('waypoints_topic').value, 10)

        self.create_subscription(Image, gp('image_topic').value,
                                 self._image_cb, qos_profile_sensor_data)
        self.create_subscription(String, gp('voice_topic').value,
                                 self._instruction_cb, 10)
        self.create_subscription(String, gp('instruction_topic').value,
                                 self._instruction_cb, 10)
        self.create_subscription(Int8, gp('awake_topic').value,
                                 self._awake_cb, 10)
        # dashboard 地图选点: 增/改/删命名航点(JSON), 持久化到 user_waypoints_file
        self.create_subscription(String, gp('waypoint_cmd_topic').value,
                                 self._waypoint_cmd_cb, 10)

        self._publish_waypoints()
        self.create_timer(3.0, self._publish_waypoints)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_action = None
        self._goal_handle = None
        if self.use_action:
            self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._last_instruction = ''
        self._last_stamp = self.get_clock().now()

        # 大模型推理放到独立工作线程, 避免长耗时推理阻塞 ROS 执行器
        # (否则推理期间收不到图像/TF/语音, 节点会"假死")
        self._task_queue = queue.Queue(maxsize=2)
        self._worker_stop = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_loop, name='vla_worker', daemon=True)
        self._worker.start()

        self.get_logger().info(
            'VLA 导航节点就绪: 模型=%s, 图像=%s, 目标发布=%s' % (
                gp('vlm_model').value, gp('image_topic').value,
                'navigate_to_pose(action)' if self.use_action else gp('goal_topic').value))

    # ----------------------------------------------------------------- helpers
    def _load_waypoints(self, configured_path):
        # 用户保存过的航点(dashboard 地图选点)优先 —— 标定一次, 重启/重编译后仍生效
        if self.user_waypoints_file and os.path.isfile(self.user_waypoints_file):
            self.get_logger().info('从用户航点文件加载: %s' % self.user_waypoints_file)
            return WaypointMap.from_yaml(self.user_waypoints_file)
        path = configured_path
        if not path:
            try:
                share = get_package_share_directory('vla_navigation')
                path = os.path.join(share, 'config', 'waypoints.yaml')
            except Exception:  # noqa: BLE001  找不到 share 目录时退化为空航点
                path = ''
        return WaypointMap.from_yaml(path)

    def _publish_waypoints(self):
        payload = {'waypoints': self.waypoint_map.to_dict_list(),
                   'file': self.user_waypoints_file}
        self.waypoints_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _waypoint_cmd_cb(self, msg):
        """dashboard 地图选点命令: {"action":"add|remove","name":...,"x","y","yaw","aliases"}.

        add(同名即覆盖)/remove 后立即换入新航点表(原子换引用, 推理线程读到的
        总是完整列表), 并整表写入 user_waypoints_file 持久化。
        """
        try:
            cmd = json.loads(msg.data or '{}')
        except ValueError:
            self._status('[航点] 命令不是合法 JSON, 已忽略')
            return
        action = str(cmd.get('action', '')).lower()
        name = str(cmd.get('name', '') or '').strip()
        if not name:
            self._status('[航点] 缺少航点名, 已忽略')
            return

        new_map = WaypointMap(list(self.waypoint_map.waypoints))
        if action in ('add', 'update', 'upsert'):
            try:
                aliases = [str(a).strip() for a in (cmd.get('aliases') or [])
                           if str(a).strip()]
                wp = Waypoint(name=name,
                              x=float(cmd.get('x')),
                              y=float(cmd.get('y')),
                              yaw=float(cmd.get('yaw', 0.0) or 0.0),
                              aliases=aliases)
            except (TypeError, ValueError):
                self._status('[航点] "%s" 坐标无效, 已忽略' % name)
                return
            verb = '更新' if new_map.upsert(wp) else '新增'
            detail = ' (x=%.2f, y=%.2f, yaw=%.2f)' % (wp.x, wp.y, wp.yaw)
        elif action in ('remove', 'delete'):
            if not new_map.remove(name):
                self._status('[航点] 删除失败: 没有名为 "%s" 的航点' % name)
                return
            verb = '删除'
            detail = ''
        else:
            self._status('[航点] 未知命令 "%s", 已忽略' % action)
            return

        self.waypoint_map = new_map
        saved = ''
        try:
            if new_map.save(self.user_waypoints_file):
                saved = ', 已存 %s' % self.user_waypoints_file
        except OSError as exc:
            saved = ', 但保存失败: %s' % exc
        self._publish_waypoints()
        self._status('[航点] %s "%s"%s, 共 %d 个%s' % (
            verb, name, detail, len(new_map.waypoints), saved))

    def _say(self, text):
        if not text:
            return
        self.tts_pub.publish(String(data=str(text)))
        self.get_logger().info('TTS: %s' % text)

    def _status(self, text):
        """发布一条可读状态(供 dashboard 时间线展示)并记录日志."""
        text = str(text)
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

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
        # 在回调里(执行器线程)快速抓取当前帧并编码, 再交给工作线程推理
        image_b64 = self._encode_image()
        if self.send_image and image_b64 is None:
            self.get_logger().warn('暂无可用摄像头图像, 本次按纯文本推理')

        try:
            self._task_queue.put_nowait((text, image_b64))
        except queue.Full:
            self.get_logger().warn('上一条指令仍在推理中, 忽略本条: %s' % text)

    def _worker_loop(self):
        """工作线程: 串行处理指令, 每条做一次大模型推理并分发动作."""
        while not self._worker_stop.is_set():
            try:
                item = self._task_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if item is None:  # 退出哨兵
                    break
                text, image_b64 = item
                self._process(text, image_b64)
            except Exception as exc:  # noqa: BLE001  单条指令异常不应拖垮线程
                self.get_logger().error('VLA 推理处理异常: %s' % exc)
            finally:
                self._task_queue.task_done()

    def _process(self, text, image_b64):
        self._status('[指令] %s' % text)
        decision = self.vlm.query(
            instruction=text,
            waypoint_text=self.waypoint_map.describe(),
            image_b64=image_b64,
        )
        self._status('[决策] %s' % json.dumps(decision, ensure_ascii=False))
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
            self._status('[停止]')
            self._say(say or '好的，停下')
            self._cancel_nav()
        elif action == 'speak':
            self._say(say or '好的')
        else:  # unknown 或非法值
            self._say(say or '抱歉，我不太明白')

    def _handle_goto(self, name, say):
        wp = self.waypoint_map.match(name)
        if wp is None:
            self._status('[导航] 未知地点: %s' % (name or ''))
            self._say('我不知道%s在哪里' % (name or '那个地点'))
            return
        z, w = wp.quaternion()
        pose = self._make_pose(self.map_frame, wp.x, wp.y, z, w)
        self._say(say or ('好的，正在前往%s' % wp.name))
        self._send_goal(pose)
        self._status('[导航] 前往 %s (x=%.2f, y=%.2f)' % (wp.name, wp.x, wp.y))

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
            self._status('[相对移动] TF 失败, 暂时无法定位')
            self._say('我暂时定位不到自己的位置')
            return

        self._say(say or '好的，马上过去')
        self._send_goal(goal)
        self._status('[相对移动] 距离=%.2fm 角度=%.1f° -> map(%.2f, %.2f)' % (
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

    def stop_worker(self):
        """通知工作线程退出并等待其结束(节点关闭时调用)."""
        self._worker_stop.set()
        try:
            self._task_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = VlaNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_worker()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
