#!/usr/bin/env python3
# coding=utf-8
"""速度指令仲裁器 (优先级 MUX).

优先级:  QR 事件  >  巡线 (line_follow)

输入:
  - ``line_follow/cmd_vel`` (geometry_msgs/Twist) 巡线节点输出的速度建议
  - ``qr_code/detected``    (std_msgs/Bool)        是否检测到二维码
  - ``qr_code/data``        (std_msgs/String)       二维码内容(用于路径选择 / 记录日志)

输出:
  - ``cmd_vel`` (geometry_msgs/Twist)  最终下发底盘的速度

状态机:
  - FOLLOW:       透传巡线速度.
  - DECELERATING: 一旦检测到二维码立即进入(高优先级), 在 ``decel_duration``
                  秒内把当前速度线性降到 0, 期间忽略巡线指令(先减速).
  - STOPPED:      保持零速(后停下); 若 ``resume_after_clear`` 为真, 当二维码
                  离开超过 ``clear_hold`` 秒后恢复巡线.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String

STATE_FOLLOW = 'FOLLOW'
STATE_DECEL = 'DECELERATING'
STATE_STOPPED = 'STOPPED'


class CmdArbiter(Node):
    def __init__(self):
        super().__init__('cmd_arbiter')

        # 参数
        self.declare_parameter('decel_duration', 1.2)       # 减速到 0 所用时间(s)
        self.declare_parameter('publish_rate', 20.0)        # cmd_vel 下发频率(Hz)
        self.declare_parameter('detect_timeout', 0.5)       # 多久没收到 True 视为二维码消失(s)
        self.declare_parameter('clear_hold', 1.0)           # 二维码离开多久后恢复巡线(s)
        self.declare_parameter('resume_after_clear', True)  # 停车后二维码消失是否恢复巡线

        self.decel_duration = self.get_parameter('decel_duration').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.detect_timeout = self.get_parameter('detect_timeout').value
        self.clear_hold = self.get_parameter('clear_hold').value
        self.resume_after_clear = self.get_parameter('resume_after_clear').value

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.follow_sub = self.create_subscription(
            Twist, 'line_follow/cmd_vel', self.follow_callback, qos)
        self.detected_sub = self.create_subscription(
            Bool, 'qr_code/detected', self.detected_callback, qos)
        self.data_sub = self.create_subscription(
            String, 'qr_code/data', self.data_callback, qos)

        self.last_follow = Twist()        # 最近一次巡线速度
        self.last_published = Twist()     # 最近一次实际下发的速度
        self.last_qr_data = ''            # 最近解码到的二维码内容

        self.qr_last_true = None          # 最近一次 detected=True 的时间(s)
        self.state = STATE_FOLLOW
        self.decel_start_time = None
        self.decel_start_cmd = Twist()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)
        self.get_logger().info('cmd_arbiter started: QR priority > line_follow')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def follow_callback(self, msg):
        self.last_follow = msg

    def detected_callback(self, msg):
        if msg.data:
            self.qr_last_true = self.now()

    def data_callback(self, msg):
        self.last_qr_data = msg.data

    def qr_active(self):
        if self.qr_last_true is None:
            return False
        return (self.now() - self.qr_last_true) <= self.detect_timeout

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

    def update(self):
        active = self.qr_active()

        if self.state == STATE_FOLLOW:
            if active:
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
                self.get_logger().info(f'QR stop reached (content="{self.last_qr_data}")')

        elif self.state == STATE_STOPPED:
            self.publish(Twist())  # 零速保持
            if self.resume_after_clear and not active:
                if (self.now() - self.qr_last_true) >= self.clear_hold:
                    self.state = STATE_FOLLOW
                    self.get_logger().info('QR cleared, resume line following')


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
