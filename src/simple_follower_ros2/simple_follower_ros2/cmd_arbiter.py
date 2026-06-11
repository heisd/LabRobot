#!/usr/bin/env python3
# coding=utf-8
"""速度指令仲裁器 (优先级 MUX) + 二维码路径动作.

优先级:  键盘/手动(最高)  >  {巡线 / KCF / YOLO}(三者平级, 谁更新鲜谁驱动)

输入:
  - ``cmd_vel_keyboard``    (geometry_msgs/Twist)   键盘遥控, 最高优先; 一旦有新指令
                            立即接管, 并在终端打印"键盘接管, 打断 X"
  - ``cmd_vel_manual``      (geometry_msgs/Twist)   手动通道(dashboard 遥控双发于此),
                            与键盘同级同处理

与自主导航(Nav2/VLA)的协同: 无任何控制源时只发一帧零速后保持静默(不再 20Hz
持续发零速抢 /cmd_vel); KCF/巡线/YOLO 活跃时由 vla_navigation 的 nav_arbiter
负责取消 navigate_to_pose 目标 —— 总优先级: 手动 > 功能模块 > Nav2/VLA。
  - ``line_follow/cmd_vel`` (geometry_msgs/Twist)   巡线速度(叠加下面 QR 路径动作)
  - ``kcf/cmd_vel``         (geometry_msgs/Twist)   KCF 跟踪速度(直接透传)
  - ``yolo/cmd_vel``        (geometry_msgs/Twist)   YOLO 跟随速度(直接透传)
  - ``qr_code/detected``    (std_msgs/Bool)         是否检测到二维码(仅巡线模式用)
  - ``qr_code/data``        (std_msgs/String)        二维码内容(仅巡线模式用)

巡线 / KCF / YOLO 三者平级: 同一时刻通常只跑一个, 仲裁器按"最近收到"选驱动源;
KCF / YOLO 直接透传, 巡线则叠加 QR 路径动作状态机. 键盘可随时打断三者(并打日志).

输出:
  - ``cmd_vel`` (geometry_msgs/Twist)  最终下发底盘的速度

二维码内容约定(大小写/前缀不敏感, 命中关键字即可):
  - ``path:left``     左转, 原地左转直到重新发现线 -> 恢复巡线
  - ``path:right``    右转, 原地右转直到重新发现线 -> 恢复巡线
  - ``path:stop``     停止, 保持停车(默认二维码移走后恢复巡线)
  - ``path:straight`` 直行, 停一下后继续巡线

状态机:
  FOLLOW       -- 透传巡线速度
  DECELERATING -- 检测到二维码立即进入(高优先级), decel_duration 秒内减速到 0(先减速)
  STOPPED      -- 零速停车(后停下), 停稳 stop_dwell 秒后按二维码内容决定动作
  TURNING      -- 原地左/右转: 先盲转 turn_min_time 秒离开路口, 再寻找线;
                  连续 line_confirm 帧发现线则恢复巡线(turn_max_time 秒安全超时)

"发现线" 的判据复用巡线节点: 当 line_follow 看到线时其 linear.x>0, 丢线时为 0,
因此无需改动巡线节点即可知道线是否重新出现.
"""

import math
import re

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String

STATE_FOLLOW = 'FOLLOW'
STATE_DECEL = 'DECELERATING'
STATE_STOPPED = 'STOPPED'
STATE_TURNING = 'TURNING'


def parse_action(data):
    """把二维码内容解析为 (action, angle).

    action: 'left' / 'right' / 'straight' / 'stop'
    angle : None 表示左右转为"转到重新发现线"(原行为);
            数字(度)表示固定转角, 例如 path:left30 -> ('left', 30.0).
    """
    text = (data or '').strip().lower()
    m = re.search(r'(left|right)\s*([0-9]+(?:\.[0-9]+)?)?', text)
    if m:
        angle = float(m.group(2)) if m.group(2) else None
        return m.group(1), angle
    if 'straight' in text or 'forward' in text:
        return 'straight', None
    # stop 或无法识别 -> 安全起见停车
    return 'stop', None


class CmdArbiter(Node):
    def __init__(self):
        super().__init__('cmd_arbiter')

        # 减速 / 停车参数
        self.declare_parameter('decel_duration', 1.2)       # 减速到 0 所用时间(s)
        self.declare_parameter('publish_rate', 20.0)        # cmd_vel 下发频率(Hz)
        self.declare_parameter('detect_timeout', 0.5)       # 多久没收到 True 视为二维码消失(s)
        self.declare_parameter('clear_hold', 1.0)           # 二维码离开多久后恢复巡线(s)
        self.declare_parameter('resume_after_clear', True)  # stop 码移走后是否恢复巡线
        self.declare_parameter('stop_dwell', 0.5)           # 停稳后再执行动作前的停留(s)
        self.declare_parameter('same_qr_cooldown', 5.0)     # 同一内容二维码的冷却(s)

        # 路径动作(左/右转)参数
        self.declare_parameter('enable_path_action', True)  # 是否执行左右转/直行动作
        self.declare_parameter('turn_angular_speed', 0.4)   # 原地转向角速度(rad/s)
        self.declare_parameter('turn_min_time', 1.0)        # 盲转时间, 先离开路口再找线(s)
        self.declare_parameter('turn_max_time', 8.0)        # 转向安全超时(s)
        self.declare_parameter('line_found_eps', 0.005)     # 判定"发现线"的 linear.x 阈值
        self.declare_parameter('line_confirm', 3)           # 连续多少帧发现线才确认
        self.declare_parameter('use_odom_turn', True)       # 固定转角是否用里程计闭环
        self.declare_parameter('odom_topic', '/odom')       # 里程计话题

        # 控制源仲裁: 键盘/手动(最高) > {巡线 / KCF / YOLO 平级}
        self.declare_parameter('keyboard_topic', 'cmd_vel_keyboard')  # 键盘遥控(最高优先)
        self.declare_parameter('manual_topic', 'cmd_vel_manual')     # 手动通道(dashboard 遥控双发于此, 与键盘同级)
        self.declare_parameter('kcf_topic', 'kcf/cmd_vel')           # KCF 跟踪(透传)
        self.declare_parameter('yolo_topic', 'yolo/cmd_vel')         # YOLO 跟随(透传)
        self.declare_parameter('external_timeout', 0.5)              # 键盘/KCF/YOLO 新鲜判定(s, 与 detect_timeout 对齐)

        g = self.get_parameter
        self.decel_duration = g('decel_duration').value
        self.publish_rate = g('publish_rate').value
        self.detect_timeout = g('detect_timeout').value
        self.clear_hold = g('clear_hold').value
        self.resume_after_clear = g('resume_after_clear').value
        self.stop_dwell = g('stop_dwell').value
        self.same_qr_cooldown = g('same_qr_cooldown').value
        self.enable_path_action = g('enable_path_action').value
        self.turn_angular_speed = g('turn_angular_speed').value
        self.turn_min_time = g('turn_min_time').value
        self.turn_max_time = g('turn_max_time').value
        self.line_found_eps = g('line_found_eps').value
        self.line_confirm = g('line_confirm').value
        self.use_odom_turn = g('use_odom_turn').value
        self.odom_topic = g('odom_topic').value
        self.keyboard_topic = g('keyboard_topic').value
        self.kcf_topic = g('kcf_topic').value
        self.yolo_topic = g('yolo_topic').value
        self.external_timeout = g('external_timeout').value

        # 防止非法频率导致除零 / 异常高频定时器
        if self.publish_rate is None or self.publish_rate < 1.0:
            self.get_logger().warn(
                f'publish_rate={self.publish_rate} invalid, clamping to 1.0 Hz')
            self.publish_rate = 1.0

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        # 当前控制源状态(给仪表盘显示: 键盘/巡线/KCF/YOLO/停车)
        self.status_pub = self.create_publisher(String, 'cmd_arbiter/status', qos)
        self.follow_sub = self.create_subscription(
            Twist, 'line_follow/cmd_vel', self.follow_callback, qos)
        self.detected_sub = self.create_subscription(
            Bool, 'qr_code/detected', self.detected_callback, qos)
        self.data_sub = self.create_subscription(
            String, 'qr_code/data', self.data_callback, qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos)
        self.keyboard_sub = self.create_subscription(
            Twist, self.keyboard_topic, self.keyboard_callback, qos)
        # 手动通道与键盘同级: dashboard 遥控双发 cmd_vel_manual, 同样可打断功能模块
        self.manual_sub = self.create_subscription(
            Twist, g('manual_topic').value, self.keyboard_callback, qos)
        self.kcf_sub = self.create_subscription(
            Twist, self.kcf_topic, self.kcf_callback, qos)
        self.yolo_sub = self.create_subscription(
            Twist, self.yolo_topic, self.yolo_callback, qos)

        self.last_follow = Twist()        # 最近一次巡线速度
        self.last_follow_time = None      # 最近一次收到巡线速度的时间
        self.last_published = Twist()     # 最近一次实际下发的速度
        self.last_qr_data = ''            # 最近解码到的二维码内容
        self.last_keyboard = Twist()      # 最近一次键盘速度
        self.last_keyboard_time = None
        self.last_kcf = Twist()           # 最近一次 KCF 速度
        self.last_kcf_time = None
        self.last_yolo = Twist()          # 最近一次 YOLO 速度
        self.last_yolo_time = None
        self.kb_engaged = False           # 当前是否键盘接管
        self.active_peer = None           # 当前驱动源标签(巡线/KCF/YOLO), 供键盘打断日志
        self.last_status = None           # 最近发布的控制源状态(给仪表盘)
        self.last_status_time = -1e9

        self.idle_silent = False          # 无控制源时是否已发过停车帧(之后静默)
        self.qr_last_true = None          # 最近一次 detected=True 的时间(s)
        self.last_handled_data = ''       # 最近一次已处理的二维码内容
        self.last_handled_time = -1e9     # 最近一次处理完成的时间(用于冷却)
        self.state = STATE_FOLLOW
        self.armed = True                 # 是否允许二维码触发(防止对同一码反复触发)
        self.decel_start_time = None
        self.decel_start_cmd = Twist()
        self.stop_time = None
        self.turn_start_time = None
        self.turn_dir = 0.0               # +1 左转, -1 右转
        self.turn_fixed = False           # True=固定转角, False=转到发现线
        self.turn_target_time = 0.0       # 固定转角(开环)需要转的时长(s)
        self.turn_target_rad = 0.0        # 固定转角目标弧度
        self.turn_safety_time = 0.0       # 固定转角安全超时(s)
        self.turn_use_odom = False        # 本次固定转角是否用里程计闭环
        self.turn_accum = 0.0             # 已累计转过的弧度(里程计)
        self.turn_prev_yaw = 0.0          # 上一次 yaw, 用于累计
        self.line_hits = 0

        # 里程计
        self.have_odom = False
        self.current_yaw = 0.0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)
        self.get_logger().info(
            '速度仲裁器启动: 键盘(最高) > {巡线/KCF/YOLO 平级}; '
            f'键盘={self.keyboard_topic} kcf={self.kcf_topic} yolo={self.yolo_topic}')

    # ------------------------------------------------------------------ utils
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def follow_callback(self, msg):
        self.last_follow = msg
        self.last_follow_time = self.now()

    def detected_callback(self, msg):
        if msg.data:
            self.qr_last_true = self.now()

    def data_callback(self, msg):
        self.last_qr_data = msg.data

    def keyboard_callback(self, msg):
        self.last_keyboard = msg
        self.last_keyboard_time = self.now()

    def kcf_callback(self, msg):
        self.last_kcf = msg
        self.last_kcf_time = self.now()

    def yolo_callback(self, msg):
        self.last_yolo = msg
        self.last_yolo_time = self.now()

    @staticmethod
    def _yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def odom_callback(self, msg):
        self.current_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def qr_active(self):
        if self.qr_last_true is None:
            return False
        return (self.now() - self.qr_last_true) <= self.detect_timeout

    def _fresh(self, t):
        """某来源最近一次时间戳是否仍新鲜(在 external_timeout 内)."""
        return t is not None and (self.now() - t) <= self.external_timeout

    def _set_status(self, label):
        """发布当前控制源状态(变化时即发, 否则 ~1Hz 心跳, 便于仪表盘显示与判活)."""
        now = self.now()
        if label != self.last_status or (now - self.last_status_time) > 1.0:
            self.status_pub.publish(String(data=label))
            self.last_status = label
            self.last_status_time = now

    def in_cooldown(self):
        """同一内容的二维码是否处于冷却期(短时间内只识别一次)."""
        if not self.last_handled_data:
            return False
        if self.last_qr_data != self.last_handled_data:
            return False
        return (self.now() - self.last_handled_time) < self.same_qr_cooldown

    def line_found(self):
        """巡线节点当前是否看到线(用其 linear.x 作为代理, 且数据要新鲜)."""
        if self.last_follow_time is None:
            return False
        if (self.now() - self.last_follow_time) > self.detect_timeout:
            return False
        return self.last_follow.linear.x > self.line_found_eps

    @staticmethod
    def scale_twist(src, factor):
        out = Twist()
        out.linear.x = src.linear.x * factor
        out.linear.y = src.linear.y * factor
        out.linear.z = src.linear.z * factor
        out.angular.x = src.angular.x * factor
        out.angular.y = src.angular.y * factor
        out.angular.z = src.angular.z * factor
        return out

    def publish(self, twist):
        self.cmd_pub.publish(twist)
        self.last_published = twist
        self.idle_silent = False   # 任何主动下发都解除"空闲静默"

    # ----------------------------------------------------------- transitions
    def enter_decel(self):
        self.state = STATE_DECEL
        self.decel_start_time = self.now()
        # 记录本次处理的二维码内容, 启动同内容冷却
        self.last_handled_data = self.last_qr_data
        self.last_handled_time = self.now()
        # 从当前实际速度开始减速, 保证平滑; 若刚好为 0 则退回巡线速度
        start = self.last_published
        if abs(start.linear.x) < 1e-6 and abs(start.angular.z) < 1e-6:
            start = self.last_follow
        self.decel_start_cmd = start
        self.get_logger().info(
            f'QR detected -> decelerate then stop (content="{self.last_qr_data}")')

    def enter_turn(self, direction, angle=None):
        self.state = STATE_TURNING
        self.turn_start_time = self.now()
        self.turn_dir = 1.0 if direction == 'left' else -1.0
        self.line_hits = 0
        if angle is not None and self.turn_angular_speed > 1e-3:
            # 固定转角
            self.turn_fixed = True
            self.turn_target_rad = math.radians(angle)
            self.turn_target_time = self.turn_target_rad / self.turn_angular_speed
            # 安全超时: 期望时长的 2 倍再加 2s, 防止里程计异常时一直打转
            self.turn_safety_time = self.turn_target_time * 2.0 + 2.0
            # 有里程计就闭环, 否则退回开环按时间
            self.turn_use_odom = bool(self.use_odom_turn and self.have_odom)
            self.turn_accum = 0.0
            self.turn_prev_yaw = self.current_yaw
            mode = 'odom' if self.turn_use_odom else 'time'
            self.get_logger().info(
                f'QR action: turn {direction} fixed {angle} deg '
                f'[{mode}] (~{self.turn_target_time:.2f}s)')
        else:
            # 转到重新发现线(原行为)
            self.turn_fixed = False
            self.turn_use_odom = False
            self.turn_target_time = 0.0
            self.get_logger().info(
                f'QR action: turn {direction} until line re-found')

    def resume_follow(self):
        self.state = STATE_FOLLOW
        # 解除武装, 必须等当前二维码彻底离开后才允许再次触发, 防止重复触发同一码
        self.armed = False
        # 动作完成后刷新冷却起点, 保证同一码在冷却期内不会被再次处理
        self.last_handled_time = self.now()
        self.get_logger().info('resume line following')

    # ------------------------------------------------------------------ loop
    def update(self):
        # 1) 键盘最高优先: 一旦有新键盘指令立即接管, 并打印"打断了谁"
        if self._fresh(self.last_keyboard_time):
            if not self.kb_engaged:
                self.kb_engaged = True
                self.get_logger().warn(
                    f'键盘接管 (cmd_vel_keyboard), 打断 {self.active_peer or "无"}')
            self._set_status('键盘' if not self.active_peer
                             else f'键盘 (打断 {self.active_peer})')
            self.publish(self.last_keyboard)
            return
        if self.kb_engaged:
            self.kb_engaged = False
            self.state = STATE_FOLLOW
            self.armed = True
            self.get_logger().info('键盘释放, 交还 巡线/KCF/YOLO')

        # 2) 巡线的 QR 路径动作(减速/停车/转向)一旦开始就必须自包含跑完, 不被巡线
        #    短暂停发或 KCF/YOLO 抢占打断; 仅在没有进行中的动作时才在三源间按"最近收到"选
        in_qr_action = self.state in (STATE_DECEL, STATE_STOPPED, STATE_TURNING)
        if not in_qr_action:
            # 巡线 / KCF / YOLO 三者平级: 选"最近收到"的那个作为当前驱动源
            sources = []
            if self._fresh(self.last_kcf_time):
                sources.append((self.last_kcf_time, 'KCF'))
            if self._fresh(self.last_yolo_time):
                sources.append((self.last_yolo_time, 'YOLO'))
            if self._fresh(self.last_follow_time):
                sources.append((self.last_follow_time, '巡线'))
            cur = max(sources)[1] if sources else None
            if cur != self.active_peer:
                if cur is not None:
                    self.get_logger().info(f'控制源 -> {cur}')
                self.active_peer = cur

            # KCF / YOLO 直接透传, 旁路 QR 状态机
            if cur == 'KCF':
                self.state = STATE_FOLLOW
                self._set_status('KCF')
                self.publish(self.last_kcf)
                return
            if cur == 'YOLO':
                self.state = STATE_FOLLOW
                self._set_status('YOLO')
                self.publish(self.last_yolo)
                return
            if cur is None:
                self._set_status('停车 (无控制源)')
                # 失去全部控制源时只发一帧零速停车, 之后保持静默 ——
                # 原先 20Hz 持续发零速会与 Nav2/VLA 的 /cmd_vel 打架,
                # 导致仲裁器空闲时机器人无法自主导航(配合 nav_arbiter 协同)。
                if not self.idle_silent:
                    self.publish(Twist())
                    self.idle_silent = True
                return
            # cur == '巡线': 落到下面的状态机
        else:
            self.active_peer = '巡线'   # QR 动作进行中, 驱动源仍记为巡线

        # 3) 巡线 / QR 路径动作状态机(FOLLOW/DECEL/STOPPED/TURNING)
        self._set_status({
            STATE_FOLLOW: '巡线', STATE_DECEL: '巡线·QR减速',
            STATE_STOPPED: '巡线·QR停车', STATE_TURNING: '巡线·QR转向',
        }.get(self.state, '巡线'))
        active = self.qr_active()

        if self.state == STATE_FOLLOW:
            # 二维码离开后重新武装
            if not active:
                self.armed = True
            # 同一内容二维码在冷却期内不再触发
            if self.armed and active and not self.in_cooldown():
                self.enter_decel()
            else:
                if active and self.in_cooldown():
                    self.get_logger().info(
                        f'QR "{self.last_qr_data}" in cooldown, ignored',
                        throttle_duration_sec=1.0)
                # 巡线数据过期(line_follow 崩溃/停发)时下发零速, 不再透传陈旧速度
                if (self.last_follow_time is None or
                        (self.now() - self.last_follow_time) > self.detect_timeout):
                    self.publish(Twist())
                else:
                    self.publish(self.last_follow)

        elif self.state == STATE_DECEL:
            elapsed = self.now() - self.decel_start_time
            if self.decel_duration <= 0.0:
                factor = 0.0
            else:
                factor = max(0.0, 1.0 - elapsed / self.decel_duration)
            self.publish(self.scale_twist(self.decel_start_cmd, factor))
            if factor <= 0.0:
                self.state = STATE_STOPPED
                self.stop_time = self.now()
                self.get_logger().info(
                    f'QR stop reached (content="{self.last_qr_data}")')

        elif self.state == STATE_STOPPED:
            self.publish(Twist())  # 零速保持
            # 先停稳一会儿
            if (self.now() - self.stop_time) < self.stop_dwell:
                return
            action, angle = parse_action(self.last_qr_data)
            if self.enable_path_action and action in ('left', 'right'):
                self.enter_turn(action, angle)
            elif self.enable_path_action and action == 'straight':
                self.resume_follow()
            else:  # stop 或未启用动作
                if self.resume_after_clear and not active and \
                        (self.now() - self.qr_last_true) >= self.clear_hold:
                    self.resume_follow()

        elif self.state == STATE_TURNING:
            turn = Twist()
            turn.angular.z = self.turn_dir * self.turn_angular_speed
            self.publish(turn)
            elapsed = self.now() - self.turn_start_time
            # 固定转角: 里程计闭环到目标角度(或开环按时间), 不等线
            if self.turn_fixed:
                target_deg = math.degrees(self.turn_target_rad)
                if self.turn_use_odom:
                    delta = self.current_yaw - self.turn_prev_yaw
                    delta = math.atan2(math.sin(delta), math.cos(delta))  # 处理 ±pi 翻转
                    self.turn_accum += delta
                    self.turn_prev_yaw = self.current_yaw
                    done_deg = math.degrees(abs(self.turn_accum))
                    self.get_logger().info(
                        f'turning [odom] {done_deg:.1f}/{target_deg:.1f} deg',
                        throttle_duration_sec=0.3)
                    if abs(self.turn_accum) >= self.turn_target_rad:
                        self.get_logger().info(
                            f'fixed turn done [odom]: turned {done_deg:.1f} deg '
                            f'(target {target_deg:.1f})')
                        self.resume_follow()
                        return
                else:
                    self.get_logger().info(
                        f'turning [time] {elapsed:.2f}/{self.turn_target_time:.2f}s '
                        f'(~{target_deg:.1f} deg)',
                        throttle_duration_sec=0.3)
                    if elapsed >= self.turn_target_time:
                        self.get_logger().info(
                            f'fixed turn done [time]: ~{target_deg:.1f} deg '
                            f'in {elapsed:.2f}s')
                        self.resume_follow()
                        return
                if elapsed >= self.turn_safety_time:
                    self.get_logger().warn(
                        f'fixed-turn timeout after {elapsed:.1f}s, resume anyway')
                    self.resume_follow()
                return
            # 寻线转向: 盲转阶段结束后才开始找线, 避免在路口原地的旧线上误判
            self.get_logger().info(
                f'turning [seek] {elapsed:.1f}s, line hits={self.line_hits}/{self.line_confirm}',
                throttle_duration_sec=0.5)
            if elapsed >= self.turn_min_time:
                if self.line_found():
                    self.line_hits += 1
                else:
                    self.line_hits = 0
                if self.line_hits >= self.line_confirm:
                    self.get_logger().info(
                        f'seek turn done: line re-found after {elapsed:.1f}s')
                    self.resume_follow()
                    return
            if elapsed >= self.turn_max_time:
                self.get_logger().warn('turn timeout, resume line following anyway')
                self.resume_follow()


def main(args=None):
    rclpy.init(args=args)
    node = CmdArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
