#!/usr/bin/env python3
"""导航仲裁节点: 手动遥控随时打断 Nav2 / VLA 的自主导航.

本仓库里 Nav2 目标有两个来源, 但都汇入 bt_navigator 的 navigate_to_pose:
  - RViz / dashboard 直接发 /goal_pose
  - vla_navigator 决策后发 /goal_pose (或 use_action 模式直接发动作目标)
因此"打断自主导航"只需一件事: 取消 navigate_to_pose 的全部目标 ——
bt_navigator 停止后 Nav2 不再发 /cmd_vel, 手动遥控独占底盘。

工作方式(目标监督式, 不需要改 Nav2 的 launch/remap):
  - 订阅手动速度话题(默认 cmd_vel_manual 与 cmd_vel_keyboard 两个):
    dashboard 遥控会同时发到 cmd_vel_manual; 键盘节点用
    `ros2 run wheeltec_robot_keyboard wheeltec_keyboard --ros-args -r cmd_vel:=cmd_vel_manual`
  - 收到任意手动速度 -> 进入 MANUAL: 立即取消 navigate_to_pose 全部目标,
    并(可选)把手动速度转发到 cmd_vel; 手动窗口为滑动 manual_timeout 秒。
  - MANUAL 期间若 Nav2 又开始输出(/cmd_vel_nav 有新数据, 例如 VLA 在
    手动期间塞了新目标), 限频重复取消 —— 手动期间自主进不来。
  - 手动窗口超时 -> 回到 AUTO: 发一帧零速兜底, 之后不干预,
    新的 /goal_pose / VLA 指令照常被 Nav2 接受。
  - 状态发布到 nav_arbiter/status (std_msgs/String, "MANUAL|AUTO: 说明"),
    变更即发 + 1s 心跳, dashboard 的 VLA 页时间线可见。

注意: 本节点不做速度混控(底盘固件按"最新收到"的目标速度执行),
只负责让自主导航在手动介入时立刻让位。
"""

import unique_identifier_msgs.msg

import rclpy
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NavArbiter(Node):

    def __init__(self):
        super().__init__('nav_arbiter')

        # 手动输入话题(可多个); dashboard 发 cmd_vel_manual,
        # 键盘节点 remap 到 cmd_vel_manual 或沿用旧仲裁器的 cmd_vel_keyboard。
        self.declare_parameter('manual_topics', ['cmd_vel_manual', 'cmd_vel_keyboard'])
        self.declare_parameter('manual_timeout', 2.0)   # 手动窗口(s, 滑动)
        self.declare_parameter('forward_manual', True)  # 手动速度是否转发到 out_topic
        self.declare_parameter('out_topic', 'cmd_vel')
        self.declare_parameter('nav_cmd_topic', 'cmd_vel_nav')  # Nav2 controller 原始输出(humble 默认存在)
        self.declare_parameter('cancel_service', 'navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('recancel_interval', 1.0)  # 手动期间重复取消的最小间隔(s)
        self.declare_parameter('status_topic', 'nav_arbiter/status')

        g = self.get_parameter
        self.manual_timeout = float(g('manual_timeout').value)
        self.forward_manual = bool(g('forward_manual').value)
        self.recancel_interval = float(g('recancel_interval').value)

        self.cmd_pub = self.create_publisher(Twist, g('out_topic').value, 10)
        self.status_pub = self.create_publisher(String, g('status_topic').value, 10)
        self.cancel_cli = self.create_client(CancelGoal, g('cancel_service').value)

        for topic in (g('manual_topics').value or []):
            if topic:
                self.create_subscription(Twist, topic, self._manual_cb, 10)
        self.create_subscription(Twist, g('nav_cmd_topic').value, self._nav_cmd_cb, 10)

        self._manual_until = None     # rclpy.time.Time, 手动窗口截止
        self._last_cancel = None
        self._last_status = ''
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            '导航仲裁就绪: 手动话题=%s, 超时=%.1fs, 取消服务=%s' % (
                list(g('manual_topics').value or []), self.manual_timeout,
                g('cancel_service').value))
        self._publish_status('AUTO: 自主导航可用')

    # ------------------------------------------------------------- callbacks
    def _manual_cb(self, msg):
        now = self.get_clock().now()
        entering = not self._manual_active(now)
        self._manual_until = now + rclpy.duration.Duration(seconds=self.manual_timeout)
        if entering:
            self.get_logger().info('手动接管: 取消 Nav2 / VLA 当前导航目标')
            self._cancel_all(now)
            self._publish_status('MANUAL: 手动接管, 已取消自主导航目标')
        if self.forward_manual:
            self.cmd_pub.publish(msg)

    def _nav_cmd_cb(self, _msg):
        # 手动期间 Nav2 又动了(例如 VLA 在此期间塞了新目标) -> 限频重复取消
        now = self.get_clock().now()
        if not self._manual_active(now):
            return
        if self._last_cancel is not None and \
                (now - self._last_cancel) < rclpy.duration.Duration(seconds=self.recancel_interval):
            return
        self.get_logger().info('手动期间检测到自主导航输出, 再次取消')
        self._cancel_all(now)

    def _tick(self):
        now = self.get_clock().now()
        if self._manual_until is not None and not self._manual_active(now):
            # 手动窗口结束: 零速兜底一帧, 把控制权还给自主导航
            self._manual_until = None
            self.cmd_pub.publish(Twist())
            self.get_logger().info('手动窗口结束, 恢复自主导航可用')
            self._publish_status('AUTO: 自主导航可用')
        elif self._manual_active(now):
            remain = (self._manual_until - now).nanoseconds / 1e9
            self._publish_status('MANUAL: 手动接管中 (%.1fs 后释放)' % remain, throttled=True)
        else:
            self._publish_status('AUTO: 自主导航可用', throttled=True)

    # --------------------------------------------------------------- helpers
    def _manual_active(self, now):
        return self._manual_until is not None and now < self._manual_until

    def _cancel_all(self, now):
        """取消 navigate_to_pose 的全部目标(零 UUID + 零时间戳 = cancel all)."""
        self._last_cancel = now
        if not self.cancel_cli.service_is_ready():
            # bt_navigator 未起或没在导航 —— 无目标可取消, 不阻塞等待
            self.get_logger().warn('取消服务不可用(Nav2 未运行或无活动目标)')
            return
        req = CancelGoal.Request()
        req.goal_info.goal_id = unique_identifier_msgs.msg.UUID()
        req.goal_info.stamp = TimeMsg()
        self.cancel_cli.call_async(req)

    _status_count = 0

    def _publish_status(self, text, throttled=False):
        # throttled: 心跳态每 1s 发一次(定时器 0.2s * 5), 变更态立即发
        if throttled:
            self._status_count += 1
            if text == self._last_status and self._status_count % 5 != 0:
                return
        self._last_status = text
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = NavArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
