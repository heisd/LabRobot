#!/usr/bin/env python3
# coding=utf-8
"""速度指令仲裁器 (优先级 MUX) + 二维码路径动作.

优先级:  QR 事件  >  巡线 (line_follow)

输入:
  - ``line_follow/cmd_vel`` (geometry_msgs/Twist) 巡线节点输出的速度建议
  - ``qr_code/detected``    (std_msgs/Bool)        是否检测到二维码
  - ``qr_code/data``        (std_msgs/String)       二维码内容(用于路径选择)

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

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String

STATE_FOLLOW = 'FOLLOW'
STATE_DECEL = 'DECELERATING'
STATE_STOPPED = 'STOPPED'
STATE_TURNING = 'TURNING'


def parse_action(data):
    """把二维码内容解析为动作: left / right / stop / straight."""
    text = (data or '').strip().lower()
    if 'left' in text:
        return 'left'
    if 'right' in text:
        return 'right'
    if 'straight' in text or 'forward' in text:
        return 'straight'
    # stop 或无法识别 -> 安全起见停车
    return 'stop'


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

        # 路径动作(左/右转)参数
        self.declare_parameter('enable_path_action', True)  # 是否执行左右转/直行动作
        self.declare_parameter('turn_angular_speed', 0.4)   # 原地转向角速度(rad/s)
        self.declare_parameter('turn_min_time', 1.0)        # 盲转时间, 先离开路口再找线(s)
        self.declare_parameter('turn_max_time', 8.0)        # 转向安全超时(s)
        self.declare_parameter('line_found_eps', 0.005)     # 判定"发现线"的 linear.x 阈值
        self.declare_parameter('line_confirm', 3)           # 连续多少帧发现线才确认

        g = self.get_parameter
        self.decel_duration = g('decel_duration').value
        self.publish_rate = g('publish_rate').value
        self.detect_timeout = g('detect_timeout').value
        self.clear_hold = g('clear_hold').value
        self.resume_after_clear = g('resume_after_clear').value
        self.stop_dwell = g('stop_dwell').value
        self.enable_path_action = g('enable_path_action').value
        self.turn_angular_speed = g('turn_angular_speed').value
        self.turn_min_time = g('turn_min_time').value
        self.turn_max_time = g('turn_max_time').value
        self.line_found_eps = g('line_found_eps').value
        self.line_confirm = g('line_confirm').value

        # 防止非法频率导致除零 / 异常高频定时器
        if self.publish_rate is None or self.publish_rate < 1.0:
            self.get_logger().warn(
                f'publish_rate={self.publish_rate} invalid, clamping to 1.0 Hz')
            self.publish_rate = 1.0

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.follow_sub = self.create_subscription(
            Twist, 'line_follow/cmd_vel', self.follow_callback, qos)
        self.detected_sub = self.create_subscription(
            Bool, 'qr_code/detected', self.detected_callback, qos)
        self.data_sub = self.create_subscription(
            String, 'qr_code/data', self.data_callback, qos)

        self.last_follow = Twist()        # 最近一次巡线速度
        self.last_follow_time = None      # 最近一次收到巡线速度的时间
        self.last_published = Twist()     # 最近一次实际下发的速度
        self.last_qr_data = ''            # 最近解码到的二维码内容

        self.qr_last_true = None          # 最近一次 detected=True 的时间(s)
        self.state = STATE_FOLLOW
        self.armed = True                 # 是否允许二维码触发(防止对同一码反复触发)
        self.decel_start_time = None
        self.decel_start_cmd = Twist()
        self.stop_time = None
        self.turn_start_time = None
        self.turn_dir = 0.0               # +1 左转, -1 右转
        self.line_hits = 0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)
        self.get_logger().info('cmd_arbiter started: QR priority > line_follow')

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

    def qr_active(self):
        if self.qr_last_true is None:
            return False
        return (self.now() - self.qr_last_true) <= self.detect_timeout

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

    # ----------------------------------------------------------- transitions
    def enter_decel(self):
        self.state = STATE_DECEL
        self.decel_start_time = self.now()
        # 从当前实际速度开始减速, 保证平滑; 若刚好为 0 则退回巡线速度
        start = self.last_published
        if abs(start.linear.x) < 1e-6 and abs(start.angular.z) < 1e-6:
            start = self.last_follow
        self.decel_start_cmd = start
        self.get_logger().info(
            f'QR detected -> decelerate then stop (content="{self.last_qr_data}")')

    def enter_turn(self, direction):
        self.state = STATE_TURNING
        self.turn_start_time = self.now()
        self.turn_dir = 1.0 if direction == 'left' else -1.0
        self.line_hits = 0
        self.get_logger().info(f'QR action: turn {direction} until line re-found')

    def resume_follow(self):
        self.state = STATE_FOLLOW
        # 解除武装, 必须等当前二维码彻底离开后才允许再次触发, 防止重复触发同一码
        self.armed = False
        self.get_logger().info('resume line following')

    # ------------------------------------------------------------------ loop
    def update(self):
        active = self.qr_active()

        if self.state == STATE_FOLLOW:
            # 二维码离开后重新武装
            if not active:
                self.armed = True
            if self.armed and active:
                self.enter_decel()
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
            action = parse_action(self.last_qr_data)
            if self.enable_path_action and action in ('left', 'right'):
                self.enter_turn(action)
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
            # 盲转阶段结束后才开始找线, 避免在路口原地的旧线上误判
            if elapsed >= self.turn_min_time:
                if self.line_found():
                    self.line_hits += 1
                else:
                    self.line_hits = 0
                if self.line_hits >= self.line_confirm:
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
