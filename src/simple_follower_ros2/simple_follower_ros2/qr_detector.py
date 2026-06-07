#!/usr/bin/env python3
# coding=utf-8
"""QR 码检测节点.

订阅相机图像, 使用 OpenCV ``QRCodeDetector`` 检测/解码二维码.
本节点只负责"看见二维码", 不直接抢占底盘, 速度裁决交给 ``cmd_arbiter``,
从而保证 "QR 事件优先级高于巡线(line_follow)".

发布话题:
  - ``qr_code/detected``   (std_msgs/Bool)    当前帧是否检测到二维码
  - ``qr_code/data``       (std_msgs/String)  解码内容(用于后续路径选择)
  - ``qr_code/area_ratio`` (std_msgs/Float32) 二维码面积 / 画面面积, 作为距离的粗略代理
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
        self.declare_parameter('min_area_ratio', 0.002)
        # 是否弹出窗口显示检测框(无显示器时请保持 False)
        self.declare_parameter('show_image', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.min_area_ratio = self.get_parameter('min_area_ratio').value
        self.show_image = self.get_parameter('show_image').value

        self.bridge = cv_bridge.CvBridge()
        self.detector = cv2.QRCodeDetector()

        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos)
        self.detected_pub = self.create_publisher(Bool, 'qr_code/detected', qos)
        self.data_pub = self.create_publisher(String, 'qr_code/data', qos)
        self.area_pub = self.create_publisher(Float32, 'qr_code/area_ratio', qos)

        self.last_data = ''
        self.get_logger().info(f'qr_detector started, subscribing image: {self.image_topic}')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:  # noqa: BLE001 - 单帧异常不应使节点崩溃
            self.get_logger().warn(f'cv_bridge conversion failed: {err}')
            return
        h, w = image.shape[:2]
        frame_area = float(h * w)

        data, points, _ = self.detector.detectAndDecode(image)

        detected = False
        area_ratio = 0.0
        if points is not None and len(points) > 0:
            pts = points.reshape(-1, 2)
            if pts.shape[0] >= 4:
                contour = pts.reshape(-1, 1, 2).astype(np.float32)
                area = abs(cv2.contourArea(contour))
                area_ratio = area / frame_area if frame_area > 0 else 0.0
                if area_ratio >= self.min_area_ratio:
                    detected = True

        # 每帧都发布检测状态, 由 cmd_arbiter 做超时去抖
        self.detected_pub.publish(Bool(data=bool(detected)))
        self.area_pub.publish(Float32(data=float(area_ratio)))

        if detected and data:
            if data != self.last_data:
                self.get_logger().info(f'QR decoded: "{data}" (area_ratio={area_ratio:.4f})')
                self.last_data = data
            self.data_pub.publish(String(data=data))
        elif not detected:
            self.last_data = ''

        if self.show_image:
            if detected and points is not None:
                poly = points.reshape(-1, 2).astype(int)
                cv2.polylines(image, [poly], True, (0, 255, 0), 2)
                label = data if data else 'QR'
                cv2.putText(image, label, tuple(poly[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('qr_detector', image)
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
