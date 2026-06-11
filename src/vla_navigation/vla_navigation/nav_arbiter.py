#!/usr/bin/env python3
"""导航仲裁节点: 手动遥控与底盘功能模块(KCF/巡线/YOLO)随时打断 Nav2 / VLA.

本仓库里 Nav2 目标有两个来源, 但都汇入 bt_navigator 的 navigate_to_pose:
  - RViz / dashboard 直接发 /goal_pose
  - vla_navigator 决策后发 /goal_pose (或 use_action 模式直接发动作目标)
因此"打断自主导航"只需一件事: 取消 navigate_to_pose 的全部目标 ——
bt_navigator 停止后 Nav2 不再发 /cmd_vel, 当前控制源独占底盘。

总优先级:  手动  >  功能模块(KCF / 巡线 / YOLO, 平级谁新鲜谁算)  >  Nav2/VLA

工作方式(目标监督式, 不需要改 Nav2 的 launch/remap):
  - 手动层: 订阅 cmd_vel_manual 与 cmd_vel_keyboard(dashboard 遥控双发前者;
    键盘节点 `--ros-args -r cmd_vel:=cmd_vel_manual`)。收到即取消
    navigate_to_pose 全部目标并(可选)转发到 cmd_vel; 滑动窗口 manual_timeout。
  - 功能模块层: 订阅 kcf/cmd_vel、yolo/cmd_vel、line_follow/cmd_vel
    (与 simple_follower_ros2 的 cmd_arbiter 同名约定)。任一话题新鲜即认为
    功能模块在驱动 -> 取消自主导航目标; 速度**不在这里转发** ——
    转发与 QR 路径动作仍由 cmd_arbiter 负责(它空闲时已改为静默,
    两个仲裁器可常开共存)。
  - 上层(手动/功能模块)活跃期间若 Nav2 又开始输出(/cmd_vel_nav 有新数据,
    例如 VLA 此时塞了新目标), 限频重复取消 —— 自主插不进来。
  - 全部空闲 -> AUTO: 不干预, 新的 /goal_pose / VLA 指令照常被 Nav2 接受。
  - 状态发布到 nav_arbiter/status (std_msgs/String, "MANUAL|FUNC|AUTO: 说明"),
    变更即发 + 1s 心跳, dashboard 的 VLA 页时间线可见。

注意: 本节点不做速度混控(底盘固件按"最新收到"的目标速度执行),
只负责让自主导航在更高优先级控制介入时立刻让位。
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
        # 底盘功能模块速度话题(与 cmd_arbiter 输入同名约定):
        # 任一话题新鲜 = 功能模块在驱动 -> 取消自主导航目标(不转发速度)。
        self.declare_parameter('func_topics', ['kcf/cmd_vel', 'yolo/cmd_vel', 'line_follow/cmd_vel'])
        self.declare_parameter('func_timeout', 1.0)     # 功能模块新鲜判定(s)
        self.declare_parameter('out_topic', 'cmd_vel')
        self.declare_parameter('nav_cmd_topic', 'cmd_vel_nav')  # Nav2 controller 原始输出(humble 默认存在)
        self.declare_parameter('cancel_service', 'navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('recancel_interval', 1.0)  # 上层活跃期间重复取消的最小间隔(s)
        self.declare_parameter('status_topic', 'nav_arbiter/status')

        g = self.get_parameter
        self.manual_timeout = float(g('manual_timeout').value)
        self.forward_manual = bool(g('forward_manual').value)
        self.func_timeout = float(g('func_timeout').value)
        self.recancel_interval = float(g('recancel_interval').value)

        self.cmd_pub = self.create_publisher(Twist, g('out_topic').value, 10)
        self.status_pub = self.create_publisher(String, g('status_topic').value, 10)
        self.cancel_cli = self.create_client(CancelGoal, g('cancel_service').value)

        for topic in (g('manual_topics').value or []):
            if topic:
                self.create_subscription(Twist, topic, self._manual_cb, 10)
        self.create_subscription(Twist, g('nav_cmd_topic').value, self._nav_cmd_cb, 10)

        # 功能模块层: 每个话题记录最近到达时间, 标签用于状态/日志显示
        self._func_stamps = {}   # label -> rclpy.time.Time | None
        for topic in (g('func_topics').value or []):
            if not topic:
                continue
            label = self._func_label(topic)
            self._func_stamps[label] = None
            self.create_subscription(
                Twist, topic,
                (lambda lab: (lambda _msg: self._func_cb(lab)))(label), 10)

        self._manual_until = None     # rclpy.time.Time, 手动窗口截止
        self._last_cancel = None
        self._last_status = ''
        self._tier = 'AUTO'           # 当前优先级层: MANUAL / FUNC / AUTO
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            '导航仲裁就绪: 手动=%s > 功能模块=%s > Nav2/VLA, 取消服务=%s' % (
                list(g('manual_topics').value or []),
                list(g('func_topics').value or []),
                g('cancel_service').value))
        self._publish_status('AUTO: 自主导航可用')

    @staticmethod
    def _func_label(topic):
        t = topic.lower()
        if 'kcf' in t:
            return 'KCF 跟踪'
        if 'yolo' in t:
            return 'YOLO 跟随'
        if 'line' in t:
            return '巡线'
        return topic

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

    def _func_cb(self, label):
        self._func_stamps[label] = self.get_clock().now()

    def _nav_cmd_cb(self, _msg):
        # 手动/功能模块活跃期间 Nav2 又动了(例如 VLA 此时塞了新目标) -> 限频重复取消
        now = self.get_clock().now()
        if not (self._manual_active(now) or self._func_active(now)):
            return
        if self._last_cancel is not None and \
                (now - self._last_cancel) < rclpy.duration.Duration(seconds=self.recancel_interval):
            return
        self.get_logger().info('高优先级控制期间检测到自主导航输出, 再次取消')
        self._cancel_all(now)

    def _tick(self):
        now = self.get_clock().now()
        manual = self._manual_active(now)
        func = None if manual else self._func_active(now)
        tier = 'MANUAL' if manual else ('FUNC' if func else 'AUTO')

        if tier != self._tier:
            if self._tier == 'MANUAL':
                # 离开手动层: 零速兜底一帧(原行为), 控制权交给下一层
                self._manual_until = None
                self.cmd_pub.publish(Twist())
            if tier == 'FUNC':
                self.get_logger().info('功能模块(%s)接管, 取消自主导航目标' % func)
                self._cancel_all(now)
            elif tier == 'AUTO':
                self.get_logger().info('恢复自主导航可用')
            self._tier = tier

        if tier == 'MANUAL':
            remain = (self._manual_until - now).nanoseconds / 1e9
            self._publish_status('MANUAL: 手动接管中 (%.1fs 后释放)' % remain, throttled=True)
        elif tier == 'FUNC':
            self._publish_status('FUNC: %s 控制中, 自主导航已让位' % func, throttled=True)
        else:
            self._publish_status('AUTO: 自主导航可用', throttled=True)

    # --------------------------------------------------------------- helpers
    def _manual_active(self, now):
        return self._manual_until is not None and now < self._manual_until

    def _func_active(self, now):
        """返回当前最新鲜且未超时的功能模块标签, 没有则 None."""
        best = None
        timeout = rclpy.duration.Duration(seconds=self.func_timeout)
        for label, stamp in self._func_stamps.items():
            if stamp is None or (now - stamp) >= timeout:
                continue
            if best is None or stamp > best[0]:
                best = (stamp, label)
        return best[1] if best else None

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
