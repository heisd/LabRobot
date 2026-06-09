#!/usr/bin/env python3
# coding=utf-8
"""YOLO 3D 跟随: 用带深度的 YOLO 3D 检测, 让小车与目标保持设定距离(默认 0.20m).

订阅 yolo_ros 的 3D 检测话题 ``/yolo/detections_3d`` (yolo_msgs/DetectionArray,
每个 detection 带 ``bbox3d``, 坐标在 ``target_frame``, 默认 base_link):
  - ``bbox3d.center.position.x`` : 前向距离(米)
  - ``bbox3d.center.position.y`` : 横向偏移(左为正)

控制:
  - 前后: 让 x 收敛到 ``desired_distance``(默认 0.20m, 启动参数可调), 比例控制 + 死区;
  - 转向: 让目标方位角 atan2(y, x) 收敛到 0(目标居中);
  - 丢目标超过 ``lost_timeout`` 自动停车(发 0 速度).

目标选择: 可选 ``target_class`` 过滤类别, 然后在候选中选离"上一帧锁定点"最近者
(无历史时选正前方最近的), 以保持跟踪同一物体. 单帧漏检会在 ``lost_timeout`` 内
沿用上次目标, 避免抖停.

诊断日志: 锁定/丢失目标、跟随状态(限频)都会打印; 出问题时给出可定位的原因——
未收到检测话题、没检测到物体、类别不匹配、或"检测到但无有效 3D 深度"(多为深度
未与彩色对齐, 需 depth_registration:=true). 方便现场排查.

发布 ``cmd_vel`` (可在 launch 中重映射). 距离在 ``target_frame``(base_link, 车体中心)
下测量; 相机有前向安装偏移时, 车头到物体的实际间隙 ≈ desired_distance − 相机前向偏移.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile
from yolo_msgs.msg import DetectionArray


class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follow')
        d = self.declare_parameter
        d('detections_topic', '/yolo/detections_3d')
        d('target_class', '')          # 空 = 任意类别
        d('desired_distance', 0.20)    # 米, 期望保持的距离(启动参数可调)
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
        self.det_topic = g('detections_topic').value
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
            DetectionArray, self.det_topic, self.on_dets, qos)
        # 发布到相对 'cmd_vel'(默认 -> /cmd_vel); launch 可重映射到 yolo/cmd_vel 交给仲裁器
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        # 在线改参: 仪表盘滑块改 desired_distance 等无需重启
        self.add_on_set_parameters_callback(self._on_set_params)

        self.lock_xy = None            # 当前锁定目标的 (x, y), 米
        self.lock_class = ''           # 锁定目标类别(日志用)
        self.last_seen = None          # 上次看到目标的时刻(秒)
        self.last_msg_time = None      # 上次收到检测话题的时刻(秒)
        self.was_following = False     # 上一拍是否在跟随(用于丢失时只警告一次)
        self.start_time = self._now()
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'yolo_follow 启动: topic={self.det_topic} class="{self.target_class or "*"}" '
            f'保持距离={self.desired:.2f}m  限速 v<={self.v_max} w<={self.w_max}  '
            f'(丢失{self.lost_timeout}s后停车)')

    def _on_set_params(self, params):
        """在线改参: 让仪表盘滑块等能实时调 desired_distance 等, 无需重启."""
        setters = {
            'desired_distance': lambda v: setattr(self, 'desired', float(v)),
            'distance_deadband': lambda v: setattr(self, 'dist_db', float(v)),
            'yaw_deadband': lambda v: setattr(self, 'yaw_db', float(v)),
            'min_range': lambda v: setattr(self, 'min_range', float(v)),
            'kp_linear': lambda v: setattr(self, 'kp_lin', float(v)),
            'kp_angular': lambda v: setattr(self, 'kp_ang', float(v)),
            'max_linear': lambda v: setattr(self, 'v_max', float(v)),
            'max_angular': lambda v: setattr(self, 'w_max', float(v)),
            'lost_timeout': lambda v: setattr(self, 'lost_timeout', float(v)),
            'target_class': lambda v: setattr(self, 'target_class', (v or '').strip()),
        }
        for p in params:
            fn = setters.get(p.name)
            if fn is None:
                continue
            try:
                fn(p.value)
            except (TypeError, ValueError) as err:
                return SetParametersResult(successful=False, reason=str(err))
            if p.name == 'desired_distance':
                self.get_logger().info(f'desired_distance 在线更新 -> {self.desired:.2f}m')
        return SetParametersResult(successful=True)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _recent(self):
        return (self.last_seen is not None
                and (self._now() - self.last_seen) <= self.lost_timeout)

    def on_dets(self, msg):
        """从一帧 3D 检测里挑出要跟随的目标, 更新锁定点; 挑不到时打印原因."""
        try:
            self.last_msg_time = self._now()
            was_recent = self._recent()         # 更新 last_seen 前先记下状态, 便于"锁定"日志
            n_total = len(msg.detections)
            n_class = 0                          # 通过类别过滤的数量
            n_valid = 0                          # 有有效 3D 深度的数量
            best = None
            best_key = None
            best_cls = ''
            for det in msg.detections:
                if self.target_class and det.class_name != self.target_class:
                    continue
                n_class += 1
                p = det.bbox3d.center.position
                x, y = p.x, p.y
                if not (math.isfinite(x) and math.isfinite(y)) or x < self.min_range:
                    continue
                n_valid += 1
                if self.lock_xy is not None:
                    key = (x - self.lock_xy[0]) ** 2 + (y - self.lock_xy[1]) ** 2
                else:
                    key = x * x + y * y
                if best_key is None or key < best_key:
                    best_key, best, best_cls = key, (x, y), det.class_name

            if best is not None:
                if not was_recent:               # 重新锁定(此前丢失/未锁)
                    self.get_logger().info(
                        f'锁定目标: class={best_cls or "?"} '
                        f'距离={best[0]:.2f}m 横向={best[1]:+.2f}m')
                self.lock_xy = best
                self.lock_class = best_cls
                self.last_seen = self._now()
                return

            # 没挑到目标: 给出原因(限频), 方便现场排查
            if n_total == 0:
                self.get_logger().info('未检测到任何物体', throttle_duration_sec=2.0)
            elif self.target_class and n_class == 0:
                self.get_logger().info(
                    f'检测到 {n_total} 个物体, 但没有类别 "{self.target_class}"',
                    throttle_duration_sec=2.0)
            elif n_valid == 0:
                self.get_logger().warn(
                    f'{n_class} 个候选都没有有效 3D 深度(x<{self.min_range}m 或 NaN): '
                    f'多半是深度未与彩色对齐——用 depth_registration:=true 启动相机, '
                    f'并确认 /camera/depth/image_raw 与对应 camera_info 在发布',
                    throttle_duration_sec=3.0)
        except Exception as err:  # noqa: BLE001 - 单帧异常不应使节点崩溃
            self.get_logger().error(f'处理检测帧出错: {err}')

    def on_timer(self):
        """固定频率输出速度: 有近期目标则跟随, 否则停车; 同时打印状态/异常."""
        tw = Twist()
        if self._recent() and self.lock_xy is not None:
            x, y = self.lock_xy
            err = x - self.desired                       # >0 太远前进, <0 太近后退
            if abs(err) > self.dist_db:
                tw.linear.x = self._clamp(self.kp_lin * err, self.v_max)
            bearing = math.atan2(y, x)                   # 目标方位角, 居中为 0
            if abs(bearing) > self.yaw_db:
                tw.angular.z = self._clamp(self.kp_ang * bearing, self.w_max)
            self.get_logger().info(
                f'跟随 class={self.lock_class or "?"} 距离={x:.2f}m(目标{self.desired:.2f}) '
                f'方位={math.degrees(bearing):+.0f}° -> v={tw.linear.x:+.2f} '
                f'w={tw.angular.z:+.2f}',
                throttle_duration_sec=1.0)
            self.was_following = True
        else:
            if self.was_following:                       # 跟随中 -> 丢失, 只警告一次
                self.get_logger().warn('目标丢失, 停车')
                self.was_following = False
            self.lock_xy = None
            self._warn_no_data()
        self.cmd_pub.publish(tw)

    def _warn_no_data(self):
        """长时间收不到/不更新检测话题时提醒(限频)."""
        if self.last_msg_time is None:
            if (self._now() - self.start_time) > 3.0:
                self.get_logger().warn(
                    f'尚未收到 {self.det_topic}: 确认已 ros2 launch wheeltec_yolo '
                    f'yolo_follow.launch.py(含 use_3d) 且话题名正确',
                    throttle_duration_sec=5.0)
        elif (self._now() - self.last_msg_time) > 2.0:
            self.get_logger().warn(
                f'{self.det_topic} 超过 2s 未更新: 3D 检测可能已停 '
                f'(相机/深度/yolo 节点?)',
                throttle_duration_sec=5.0)

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
