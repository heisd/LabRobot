#!/usr/bin/env python3

import random
import time

import cv2
import cv_bridge
import numpy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

last_erro = 0

col_black = (0, 0, 0, 180, 255, 46)
col_red = (0, 100, 80, 10, 255, 255)
col_blue = (100, 43, 46, 124, 255, 255)
col_green = (35, 43, 46, 77, 255, 255)
col_yellow = (26, 43, 46, 34, 255, 255)

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


def nothing(_):
    pass


class RandomTurnFollower(Node):
    def __init__(self):
        super().__init__('random_turn_follower')
        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.twist = Twist()
        self.ui_inited = False

        # fork decision state
        self.fork_confirm_count = 0
        self.fork_confirm_need = 3
        self.branch_choice = None  # 'left' or 'right'
        self.branch_hold_count = 0
        self.branch_hold_need = 8

    def _pick_color_threshold(self):
        m = cv2.getTrackbarPos(Switch, 'Adjust_hsv')
        if m == 0:
            color = col_red
        elif m == 1:
            color = col_green
        elif m == 2:
            color = col_blue
        elif m == 3:
            color = col_yellow
        elif m == 4:
            color = col_black
        else:
            color = (0, 0, 0, 255, 255, 255)
        return color[0], color[1], color[2], color[3], color[4], color[5]

    def image_callback(self, msg):
        global last_erro

        if not self.ui_inited:
            cv2.namedWindow('Adjust_hsv', cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch, 'Adjust_hsv', 0, 4, nothing)
            self.ui_inited = True

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        kernel = numpy.ones((5, 5), numpy.uint8)
        hsv_erode = cv2.erode(hsv, kernel, iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode, kernel, iterations=1)

        lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = self._pick_color_threshold()
        mask = cv2.inRange(hsv_dilate, (lowerbH, lowerbS, lowerbV), (upperbH, upperbS, upperbV))

        h, w, _ = image.shape
        search_top = h - 30
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        valid_components = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 30:
                valid_components.append((
                    i,
                    area,
                    centroids[i][0],
                    centroids[i][1],
                    stats[i, cv2.CC_STAT_TOP],
                    stats[i, cv2.CC_STAT_HEIGHT],
                ))

        cx = None
        cy = None

        if len(valid_components) >= 2:
            xs = [comp[2] for comp in valid_components]
            horizontal_gap = max(xs) - min(xs)
            near_bottom_components = 0
            for comp in valid_components:
                comp_bottom = comp[4] + comp[5]
                if comp_bottom >= (h - 10):
                    near_bottom_components += 1
            is_fork_scene = horizontal_gap > (w * 0.20) and near_bottom_components >= 2

            if is_fork_scene:
                self.fork_confirm_count += 1
            else:
                self.fork_confirm_count = 0
                self.branch_choice = None
                self.branch_hold_count = 0

            if self.fork_confirm_count >= self.fork_confirm_need:
                if self.branch_choice is None:
                    self.branch_choice = random.choice(['left', 'right'])
                    self.branch_hold_count = 0
                    self.get_logger().info(f'Fork detected: random branch -> {self.branch_choice}')

                if self.branch_choice == 'left':
                    target_comp = min(valid_components, key=lambda x: x[2])
                else:
                    target_comp = max(valid_components, key=lambda x: x[2])
                cx = int(target_comp[2])
                cy = int(target_comp[3])

                self.branch_hold_count += 1
                if self.branch_hold_count >= self.branch_hold_need:
                    self.fork_confirm_count = 0
                    self.branch_choice = None
                    self.branch_hold_count = 0
            else:
                m = cv2.moments(mask)
                if m['m00'] > 0:
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
        else:
            self.fork_confirm_count = 0
            self.branch_choice = None
            self.branch_hold_count = 0
            m = cv2.moments(mask)
            if m['m00'] > 0:
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])

        if cx is not None and cy is not None:
            erro = cx - w / 2
            d_erro = erro - last_erro
            self.twist.linear.x = 0.11
            if abs(erro) < 8:
                self.twist.angular.z = 0.0
            else:
                self.twist.angular.z = -(float(erro) * 0.0011 - float(d_erro) * 0.0000)
            last_erro = erro
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow('Adjust_hsv', mask)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    print('start random-turn patrolling')
    follower = RandomTurnFollower()
    while rclpy.ok():
        rclpy.spin_once(follower)
        time.sleep(0.1)

    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
