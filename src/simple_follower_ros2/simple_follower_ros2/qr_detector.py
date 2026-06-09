#!/usr/bin/env python3
# coding=utf-8
"""QR 码检测节点.

订阅相机图像, 检测/解码二维码. 本节点只负责"看见二维码", 不直接抢占底盘,
速度裁决交给 ``cmd_arbiter``, 从而保证 "QR 事件优先级高于巡线(line_follow)".

检测后端(自动选择, 越靠前越鲁棒):
  1) ``pyzbar`` (ZBar)         —— 对角度/弯曲/密集二维码远比 OpenCV 鲁棒(可选依赖)
  2) ``cv2.QRCodeDetector``    —— OpenCV 自带, 无需额外安装

  安装 pyzbar(强烈建议, 现场识别率高很多)::
      sudo apt install libzbar0
      pip3 install pyzbar

为避免"检测过程本身"打断/卡顿巡线:
  - 抽帧 ``detect_every_n``: 每 N 帧检测一次, 把算力让给巡线;
  - 缩放 ``detect_scale``: 检测前可缩小图像(默认 1.0=不缩放, 保证识别率;
    若 CPU 吃紧可调小, 但密集二维码可能识别不到);
  - 解码 + 连续确认 ``min_consecutive``: 必须连续多帧成功解码才算确认,
    单帧噪声不会触发 -> "只是在检测"时不会停车, 只有确认到二维码才停.

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
        self.declare_parameter('min_area_ratio', 0.005)      # 面积下限, 过滤远处误检
        self.declare_parameter('show_image', False)          # 可视化窗口
        self.declare_parameter('detect_every_n', 3)          # 每 N 帧检测一次
        self.declare_parameter('detect_scale', 1.0)          # 检测前缩放(1.0=不缩放)
        self.declare_parameter('min_consecutive', 3)         # 连续确认帧数, 防误停
        self.declare_parameter('publish_debug', True)        # 发布可视化图给 web_video_server(浏览器)
        self.declare_parameter('debug_topic', 'qr_code/debug_image')

        g = self.get_parameter
        self.image_topic = g('image_topic').value
        self.min_area_ratio = g('min_area_ratio').value
        self.show_image = g('show_image').value
        self.publish_debug = g('publish_debug').value
        debug_topic = g('debug_topic').value
        self.detect_every_n = max(1, int(g('detect_every_n').value))
        self.detect_scale = g('detect_scale').value
        if not (0.1 <= self.detect_scale <= 1.0):
            self.detect_scale = 1.0
        self.min_consecutive = max(1, int(g('min_consecutive').value))

        self.bridge = cv_bridge.CvBridge()
        self.detector = cv2.QRCodeDetector()

        # 可选的 pyzbar 后端(更鲁棒)
        try:
            from pyzbar import pyzbar as _pyzbar
            self._pyzbar = _pyzbar
            self.get_logger().info('QR backend: pyzbar (robust)')
        except Exception:  # noqa: BLE001 - 没装就回退
            self._pyzbar = None
            self.get_logger().warn(
                'pyzbar not available, falling back to cv2.QRCodeDetector; '
                'install with: sudo apt install libzbar0 && pip3 install pyzbar')

        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos)
        self.detected_pub = self.create_publisher(Bool, 'qr_code/detected', qos)
        self.data_pub = self.create_publisher(String, 'qr_code/data', qos)
        self.area_pub = self.create_publisher(Float32, 'qr_code/area_ratio', qos)
        # 调试可视化图(给 web_video_server -> 浏览器仪表盘看). 不依赖本地显示器.
        self.debug_pub = (self.create_publisher(Image, debug_topic, qos)
                          if self.publish_debug else None)

        self.frame_idx = 0
        self.consec = 0          # 连续成功解码计数
        self.last_data = ''
        self.get_logger().info(
            f'qr_detector started (image={self.image_topic}, '
            f'every_n={self.detect_every_n}, scale={self.detect_scale}, '
            f'confirm={self.min_consecutive})')

    def _detect(self, gray):
        """在(可能已缩放的)灰度图上检测二维码.

        返回 (data, points, area_ratio); points 为该灰度图坐标系下的角点(可能为 None).
        """
        h, w = gray.shape[:2]
        frame_area = float(h * w) or 1.0

        # 1) pyzbar: 只返回已成功解码的二维码, 鲁棒性好
        if self._pyzbar is not None:
            for sym in self._pyzbar.decode(gray):
                if sym.type != 'QRCODE':
                    continue
                data = sym.data.decode('utf-8', errors='replace')
                if len(sym.polygon) >= 4:
                    poly = np.array([[p.x, p.y] for p in sym.polygon], dtype=np.float32)
                    area = abs(cv2.contourArea(poly.reshape(-1, 1, 2)))
                else:
                    r = sym.rect
                    poly = np.array([[r.left, r.top],
                                     [r.left + r.width, r.top],
                                     [r.left + r.width, r.top + r.height],
                                     [r.left, r.top + r.height]], dtype=np.float32)
                    area = float(r.width * r.height)
                return data, poly, area / frame_area
            return '', None, 0.0

        # 2) OpenCV: 原图与 Otsu 二值化两种输入, 各试单码/多码;
        #    只接受"成功解码"的结果, 避免把背景里的暗色方块误当二维码
        for src in (gray, self._otsu(gray)):
            data, points = self._opencv_try(src)
            if data:
                pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
                if pts.shape[0] >= 4:
                    area = abs(cv2.contourArea(pts.reshape(-1, 1, 2)))
                else:
                    area = 0.0
                return data, pts, area / frame_area
        return '', None, 0.0

    @staticmethod
    def _otsu(gray):
        """Otsu 二值化, 缓解光照不均 / 轻微模糊."""
        return cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    def _opencv_try(self, img):
        """OpenCV 单码 + 多码尝试, 仅返回成功解码的 (data, points)."""
        data, points, _ = self.detector.detectAndDecode(img)
        if data:
            return data, points
        try:
            ok, datas, pts_multi, _ = self.detector.detectAndDecodeMulti(img)
        except cv2.error:
            ok = False
        if ok and datas is not None and pts_multi is not None:
            for d, p in zip(datas, pts_multi):
                if d:
                    return d, p
        return '', None

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

        # 缩放(默认不缩放, 保证识别率), 再转灰度供检测
        if self.detect_scale < 0.999:
            small = cv2.resize(image, None, fx=self.detect_scale,
                               fy=self.detect_scale, interpolation=cv2.INTER_AREA)
        else:
            small = image
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        data, points, area_ratio = self._detect(gray)

        # 必须"成功解码"且二维码足够大, 才算一次有效命中; 连续多帧才确认
        valid_hit = bool(data) and area_ratio >= self.min_area_ratio
        if valid_hit:
            self.consec += 1
        else:
            self.consec = 0
        confirmed = self.consec >= self.min_consecutive

        # 只要成功解码就打日志(限频), 方便现场确认"到底有没有检测到"
        if data:
            progress = min(self.consec, self.min_consecutive)
            self.get_logger().info(
                f'QR seen: "{data}" area={area_ratio:.4f} '
                f'(min={self.min_area_ratio}) confirm={progress}/{self.min_consecutive}',
                throttle_duration_sec=0.5)

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

        if self.publish_debug or self.show_image:
            vis = self._render_debug(image, points, data, area_ratio, valid_hit, confirmed)
            if self.debug_pub is not None:
                try:
                    out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                    out.header = msg.header
                    self.debug_pub.publish(out)
                except Exception as err:  # noqa: BLE001 - 单帧发布失败不应崩溃
                    self.get_logger().warn(f'debug image publish failed: {err}')
            if self.show_image:
                cv2.imshow('QR Check', vis)
                cv2.waitKey(3)

    def _render_debug(self, image, points, data, area_ratio, valid_hit, confirmed):
        """在 image 上画 QR 检测可视化(状态/解码内容/确认进度/检测框), 返回该图.

        既发布给 web_video_server(浏览器仪表盘), 也用于可选的本地 cv2 窗口.
        """
        if confirmed:
            status, color = 'CONFIRMED', (0, 255, 0)      # 绿: 已确认
        elif valid_hit:
            status, color = 'detecting', (0, 255, 255)    # 黄: 命中但未确认
        else:
            status, color = 'searching', (0, 0, 255)      # 红: 未检测到
        if points is not None and len(points) > 0:
            poly = (np.asarray(points).reshape(-1, 2) / self.detect_scale).astype(int)
            cv2.polylines(image, [poly], True, color, 2)
        progress = min(self.consec, self.min_consecutive)
        backend = 'pyzbar' if self._pyzbar is not None else 'opencv'
        hud1 = f'{status}  data="{data}"  [{backend}]'
        hud2 = f'area={area_ratio:.4f}  confirm={progress}/{self.min_consecutive}'
        cv2.putText(image, hud1, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(image, hud2, (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return image


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
