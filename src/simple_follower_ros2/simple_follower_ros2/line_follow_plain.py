#!/usr/bin/env python3
# coding=utf-8
"""纯巡线节点(无分叉处理).

与 ``line_follow.py`` / ``line_follow_node.py`` 的区别: 去掉了所有分叉判断
(连通域选择、左叉确认、随机左右等), 只做最朴素的
"颜色阈值 -> 底部条带质心 -> PID" 巡线.

路口的左右转交给二维码(配合 ``cmd_arbiter`` 的固定转角 / 寻线转角)决定,
所以这里完全不需要分叉逻辑.

发布 ``cmd_vel``(在 launch 中可重映射给 ``cmd_arbiter``, 例如
remappings=[('cmd_vel', 'line_follow/cmd_vel')]).
"""

import time

import cv2
import cv_bridge
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

col_black = (0, 0, 0, 180, 255, 46)
col_red = (0, 100, 80, 10, 255, 255)
col_blue = (100, 43, 46, 124, 255, 255)
col_green = (35, 43, 46, 77, 255, 255)
col_yellow = (26, 43, 46, 34, 255, 255)

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


def nothing(_):
    pass


class PlainFollower(Node):
    def __init__(self):
        super().__init__('line_follow_plain')
        # show_image: 本地 cv2 窗口(默认关, 机器人无显示器也能跑);
        # publish_debug: 把巡线掩码发布成 Image 给 web_video_server(浏览器);
        # line_color: 无本地 trackbar 时用哪种颜色(0:Red 1:Green 2:Blue 3:Yellow 4:Black).
        self.declare_parameter('show_image', False)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_topic', 'line_follow/debug_image')
        self.declare_parameter('line_color', 0)
        g = self.get_parameter
        self.show_image = g('show_image').value
        self.publish_debug = g('publish_debug').value
        debug_topic = g('debug_topic').value
        self.line_color = int(g('line_color').value)

        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.debug_pub = (self.create_publisher(Image, debug_topic, qos)
                          if self.publish_debug else None)
        self.twist = Twist()
        self.ui_inited = False
        self.cx_filtered = None
        self.last_erro = 0.0

    def _pick_color(self):
        # 有本地窗口时用 trackbar, 否则用 line_color 参数(headless 友好).
        if self.show_image and self.ui_inited:
            m = cv2.getTrackbarPos(Switch, 'Adjust_hsv')
        else:
            m = self.line_color
        table = {0: col_red, 1: col_green, 2: col_blue, 3: col_yellow, 4: col_black}
        return table.get(m, (0, 0, 0, 255, 255, 255))

    def image_callback(self, msg):
        if self.show_image and not self.ui_inited:
            cv2.namedWindow('Adjust_hsv', cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch, 'Adjust_hsv', 0, 4, nothing)
            self.ui_inited = True

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        hsv = cv2.dilate(cv2.erode(hsv, kernel, iterations=1), kernel, iterations=1)

        lH, lS, lV, uH, uS, uV = self._pick_color()
        mask = cv2.inRange(hsv, (lH, lS, lV), (uH, uS, uV))

        h, w = mask.shape[:2]
        search_top = h - 30
        mask[0:search_top, 0:w] = 0  # 只看底部一条带, 避免远处干扰

        moments = cv2.moments(mask)
        if moments['m00'] > 0:
            cx = moments['m10'] / moments['m00']
            # 一阶低通, 抑制抖动
            if self.cx_filtered is None:
                self.cx_filtered = cx
            else:
                self.cx_filtered = 0.7 * self.cx_filtered + 0.3 * cx
            erro = self.cx_filtered - w / 2.0
            self.twist.linear.x = 0.11
            if abs(erro) < 8:
                self.twist.angular.z = 0.0
            else:
                self.twist.angular.z = -(erro * 0.0011)
            self.last_erro = erro
        else:
            # 丢线: 停车(交给上层/二维码决定下一步)
            self.cx_filtered = None
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)

        # 把巡线掩码转发给浏览器(web_video_server), 并可选本地窗口.
        if self.debug_pub is not None:
            try:
                out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                out.header = msg.header
                self.debug_pub.publish(out)
            except Exception as err:  # noqa: BLE001 - 单帧发布失败不应崩溃
                self.get_logger().warn(f'debug image publish failed: {err}')
        if self.show_image:
            cv2.imshow('Adjust_hsv', mask)
            cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    print('start plain line following (no fork)')
    node = PlainFollower()
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
