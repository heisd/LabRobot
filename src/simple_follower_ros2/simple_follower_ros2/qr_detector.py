#!/usr/bin/env python3
# coding=utf-8
"""QR 码检测节点.

订阅相机图像, 使用 OpenCV ``QRCodeDetector`` 检测/解码二维码.
本节点只负责"看见二维码", 不直接抢占底盘, 速度裁决交给 ``cmd_arbiter``,
从而保证 "QR 事件优先级高于巡线(line_follow)".

设计要点(为避免"检测过程本身"打断/卡顿巡线):
  1) 抽帧检测 ``detect_every_n``: 只在每 N 帧里跑一次检测, 把算力让给巡线,
     避免两个节点抢同一路图像的 CPU 导致一卡一卡;
  2) 缩小检测 ``detect_scale``: 先把图像缩小再检测, QRCodeDetector 更快;
  3) 解码 + 连续确认 ``min_consecutive``: 必须连续多帧"成功解码"出内容才算确认,
     单帧噪声 / 偶发误检不会触发, 因此"只是在检测"时不会让小车停车,
     只有真正确认到二维码命令时 cmd_arbiter 才会减速停车.

发布话题:
  - ``qr_code/detected``   (std_msgs/Bool)    是否"确认"检测到二维码
  - ``qr_code/data``       (std_msgs/String)  解码内容(用于路径选择)
  - ``qr_code/area_ratio`` (std_msgs/Float32) 二维码面积 / 画面面积(距离粗略代理, 与缩放无关)
"""

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')

        # 参数
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        # 二维码面积占画面比例的下限, 过滤远处误检的噪点
        self.declare_parameter('min_area_ratio', 0.005)
        # 是否弹出窗口显示检测框(无显示器时请保持 False)
        self.declare_parameter('show_image', False)
        # 抽帧: 每 N 帧检测一次, 越大越省 CPU(对巡线越友好)
        self.declare_parameter('detect_every_n', 3)
        # 缩放: 检测前把图像缩小到该比例(0.1~1.0), 越小越快
        self.declare_parameter('detect_scale', 0.5)
        # 连续确认帧数: 连续多少个"已处理帧"成功解码才算确认, 防止误停
        self.declare_parameter('min_consecutive', 3)

        g = self.get_parameter
        self.image_topic = g('image_topic').value
        self.min_area_ratio = g('min_area_ratio').value
        self.show_image = g('show_image').value
        self.detect_every_n = max(1, int(g('detect_every_n').value))
        self.detect_scale = g('detect_scale').value
        if not (0.1 <= self.detect_scale <= 1.0):
            self.detect_scale = 1.0
        self.min_consecutive = max(1, int(g('min_consecutive').value))

        self.bridge = cv_bridge.CvBridge()
        self.detector = cv2.QRCodeDetector()

        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos)
        self.detected_pub = self.create_publisher(Bool, 'qr_code/detected', qos)
        self.data_pub = self.create_publisher(String, 'qr_code/data', qos)
        self.area_pub = self.create_publisher(Float32, 'qr_code/area_ratio', qos)

        self.frame_idx = 0
        self.consec = 0          # 连续成功解码计数
        self.last_data = ''
        self.get_logger().info(
            f'qr_detector started (image={self.image_topic}, '
            f'every_n={self.detect_every_n}, scale={self.detect_scale}, '
            f'confirm={self.min_consecutive})')

    def image_callback(self, msg):
        # 抽帧: 只处理每 N 帧中的一帧, 把算力让给巡线
        self.frame_idx += 1
        if (self.frame_idx % self.detect_every_n) != 0:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:  # noqa: BLE001 - 单帧异常不应使节点崩溃
            self.get_logger().warn(f'cv_bridge conversion failed: {err}')
            return

        # 缩小后再检测, 加速 QRCodeDetector
        if self.detect_scale < 0.999:
            small = cv2.resize(image, None, fx=self.detect_scale,
                               fy=self.detect_scale, interpolation=cv2.INTER_AREA)
        else:
            small = image
        h, w = small.shape[:2]
        frame_area = float(h * w)

        data, points, _ = self.detector.detectAndDecode(small)

        area_ratio = 0.0
        if points is not None and len(points) > 0:
            pts = points.reshape(-1, 2)
            if pts.shape[0] >= 4:
                contour = pts.reshape(-1, 1, 2).astype(np.float32)
                area = abs(cv2.contourArea(contour))
                area_ratio = area / frame_area if frame_area > 0 else 0.0

        # 必须"成功解码"且二维码足够大, 才算一次有效命中
        valid_hit = bool(data) and area_ratio >= self.min_area_ratio
        if valid_hit:
            self.consec += 1
        else:
            self.consec = 0

        # 连续多帧命中才"确认"; 只有确认才会让 cmd_arbiter 停车
        confirmed = self.consec >= self.min_consecutive

        self.detected_pub.publish(Bool(data=bool(confirmed)))
        self.area_pub.publish(Float32(data=float(area_ratio)))
        if confirmed:
            if data != self.last_data:
                self.get_logger().info(
                    f'QR confirmed: "{data}" (area_ratio={area_ratio:.4f})')
                self.last_data = data
            self.data_pub.publish(String(data=data))
        elif self.consec == 0:
            self.last_data = ''

        if self.show_image:
            self._show_gui(image, points, data, area_ratio, valid_hit, confirmed)

    def _show_gui(self, image, points, data, area_ratio, valid_hit, confirmed):
        """QR 检测可视化窗口: 状态 / 解码内容 / 确认进度 / 检测框."""
        if confirmed:
            status, color = 'CONFIRMED', (0, 255, 0)      # 绿: 已确认
        elif valid_hit:
            status, color = 'detecting', (0, 255, 255)    # 黄: 命中但未确认
        else:
            status, color = 'searching', (0, 0, 255)      # 红: 未检测到
        if points is not None and len(points) > 0:
            poly = (points.reshape(-1, 2) / self.detect_scale).astype(int)
            cv2.polylines(image, [poly], True, color, 2)
        progress = min(self.consec, self.min_consecutive)
        hud1 = f'{status}  data="{data}"'
        hud2 = f'area={area_ratio:.4f}  confirm={progress}/{self.min_consecutive}'
        cv2.putText(image, hud1, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(image, hud2, (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow('QR Check', image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
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
