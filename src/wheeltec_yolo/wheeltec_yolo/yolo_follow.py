#!/usr/bin/env python3
# coding=utf-8
"""YOLO 3D 跟随: 用带深度的 YOLO 3D 检测, 让小车与目标保持设定距离(默认 0.20m).

订阅 yolo_ros 的 3D 检测话题 ``/yolo/detections_3d`` (yolo_msgs/DetectionArray,
每个 detection 带 ``bbox3d``, 坐标在 ``target_frame``, 默认 base_link):
  - ``bbox3d.center.position.x`` : 前向距离(米)
  - ``bbox3d.center.position.y`` : 横向偏移(左为正)

控制:
  - 前后: 让 x 收敛到 ``desired_distance``(默认 0.20m), 比例控制 + 死区;
  - 转向: 让目标方位角 atan2(y, x) 收敛到 0(目标居中);
  - 丢目标超过 ``lost_timeout`` 自动停车(发 0 速度).

目标选择: 可选 ``target_class`` 过滤类别, 然后在候选中选离"上一帧锁定点"最近者
(无历史时选正前方最近的), 以保持跟踪同一物体. 单帧漏检会在 ``lost_timeout`` 内
沿用上次目标, 避免抖停.

发布 ``cmd_vel`` (可在 launch 中重映射, 例如交给 cmd_arbiter / 直接给底盘).

注意: 距离在 ``target_frame``(base_link, 车体中心)下测量; 相机有前向安装偏移时,
车头到物体的实际间隙 ≈ desired_distance - 相机前向偏移, 现场可据此微调
``desired_distance``.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from yolo_msgs.msg import DetectionArray


class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follow')
        d = self.declare_parameter
        d('detections_topic', '/yolo/detections_3d')
        d('cmd_vel_topic', 'cmd_vel')
        d('target_class', '')          # 空 = 任意类别
        d('desired_distance', 0.20)    # 米, 期望保持的距离
        d('distance_deadband', 0.03)   # 米, 距离误差死区内不前后动
        d('yaw_deadband', 0.08)        # 弧度, 方位角死区内不转
        d('min_range', 0.05)           # 米, 小于此认为无有效深度
        d('kp_linear', 0.6)
        d('kp_angular', 1.5)
        d('max_linear', 0.15)          # m/s
        d('max_angular', 0.6)          # rad/s
        d('lost_timeout', 0.5)         # 秒, 丢目标多久后停车
        d('control_rate', 20.0)        # Hz

        g = self.get_parameter
        self.target_class = (g('target_class').value or '').strip()
        self.desired = float(g('desired_distance').value)
        self.dist_db = float(g('distance_deadband').value)
        self.yaw_db = float(g('yaw_deadband').value)
        self.min_range = float(g('min_range').value)
        self.kp_lin = float(g('kp_linear').value)
        self.kp_ang = float(g('kp_angular').value)
        self.v_max = float(g('max_linear').value)
        self.w_max = float(g('max_angular').value)
        self.lost_timeout = float(g('lost_timeout').value)
        rate = max(1.0, float(g('control_rate').value))

        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(
            DetectionArray, g('detections_topic').value, self.on_dets, qos)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic').value, qos)

        self.lock_xy = None            # 当前锁定目标的 (x, y), 米
        self.last_seen = None          # 上次看到目标的时刻(秒)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'yolo_follow: topic={g("detections_topic").value} '
            f'class="{self.target_class or "*"}" keep={self.desired:.2f}m '
            f'(v<={self.v_max}, w<={self.w_max})')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_dets(self, msg):
        """从一帧 3D 检测里挑出要跟随的目标, 更新锁定点."""
        best = None
        best_key = None
        for det in msg.detections:
            if self.target_class and det.class_name != self.target_class:
                continue
            p = det.bbox3d.center.position
            x, y = p.x, p.y
            # 必须是有效 3D(投影到深度成功): 有限且在前方
            if not (math.isfinite(x) and math.isfinite(y)) or x < self.min_range:
                continue
            # 关联: 选离上一帧锁定点最近者; 无历史则选最近(前向距离最小)者
            if self.lock_xy is not None:
                key = (x - self.lock_xy[0]) ** 2 + (y - self.lock_xy[1]) ** 2
            else:
                key = x * x + y * y
            if best_key is None or key < best_key:
                best_key, best = key, (x, y)

        if best is not None:
            self.lock_xy = best
            self.last_seen = self._now()

    def on_timer(self):
        """固定频率输出速度: 有近期目标则跟随, 否则停车."""
        tw = Twist()
        recent = (self.last_seen is not None
                  and (self._now() - self.last_seen) <= self.lost_timeout)
        if recent and self.lock_xy is not None:
            x, y = self.lock_xy
            err = x - self.desired                       # >0 太远前进, <0 太近后退
            if abs(err) > self.dist_db:
                tw.linear.x = self._clamp(self.kp_lin * err, self.v_max)
            bearing = math.atan2(y, x)                   # 目标方位角, 居中为 0
            if abs(bearing) > self.yaw_db:
                tw.angular.z = self._clamp(self.kp_ang * bearing, self.w_max)
        else:
            self.lock_xy = None                          # 丢目标: 清锁定, 停车
        self.cmd_pub.publish(tw)

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))


def main(args=None):
    rclpy.init(args=args)
    node = YoloFollower()
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
