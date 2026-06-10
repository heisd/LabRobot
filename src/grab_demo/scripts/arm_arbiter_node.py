#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂抓取仲裁节点 arm_arbiter.

优先级: 手动操作(manual) > 自动抓取(auto = YOLO / KCF / HSV)。

职责(策略权威, 单一来源):
  - 手动接管 /arm_arbiter/manual_takeover (std_srvs/Trigger):
      立刻脉冲 /arm_arbiter/abort=True 打断正在运行的自动抓取,
      并把 /arm_arbiter/manual_active 置 True(latched), 在释放前拒绝新的自动抓取。
  - 释放      /arm_arbiter/manual_release (std_srvs/Trigger):
      manual_active=False, 交还控制权, 自动抓取可继续受理。
  - /arm_arbiter/state (std_msgs/String, latched): 当前 owner(idle/manual), 供 Dashboard 显示。

设计要点: 自动抓取节点(closed_loop_grab_node)只需 **订阅** abort 与 manual_active 两个话题
即可协作, 无需反向调用本节点 —— 降低耦合, 仲裁缺席时自动抓取也能降级照常运行。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class ArmArbiter(Node):
    def __init__(self):
        super().__init__("arm_arbiter")

        # latched(transient_local): 晚加入的订阅者也能拿到最新状态
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.manual_active = False
        self.owner = "idle"          # idle / manual

        self.manual_pub = self.create_publisher(Bool, "/arm_arbiter/manual_active", latched)
        self.abort_pub = self.create_publisher(Bool, "/arm_arbiter/abort", 10)
        self.state_pub = self.create_publisher(String, "/arm_arbiter/state", latched)
        self._publish_manual()
        self._publish_state()

        self.create_service(Trigger, "/arm_arbiter/manual_takeover", self._takeover)
        self.create_service(Trigger, "/arm_arbiter/manual_release", self._release)
        # 周期刷新 state(便于无 latched 的工具查看)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info("arm_arbiter 已启动: 手动优先, 可随时打断自动抓取")

    # ------------------------------------------------------------------
    def _takeover(self, _req, res):
        self.manual_active = True
        self.owner = "manual"
        self._publish_manual()
        self.abort_pub.publish(Bool(data=True))   # 脉冲: 打断正在运行的自动抓取
        self._publish_state()
        self.get_logger().warn("手动接管: 已请求打断自动抓取, 并暂停受理自动抓取")
        res.success = True
        res.message = "manual takeover: auto grab aborted & blocked"
        return res

    def _release(self, _req, res):
        was_manual = self.manual_active
        self.manual_active = False
        self.owner = "idle"
        self._publish_manual()
        self._publish_state()
        self.get_logger().info("手动释放: 交还控制权, 自动抓取可继续受理")
        res.success = True
        res.message = "released" if was_manual else "already idle"
        return res

    def _publish_manual(self):
        self.manual_pub.publish(Bool(data=self.manual_active))

    def _publish_state(self):
        self.state_pub.publish(String(data=self.owner))


def main(args=None):
    rclpy.init(args=args)
    node = ArmArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
