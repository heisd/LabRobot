#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import time
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
import cv_bridge
from geometry_msgs.msg import Twist
last_erro=0
tmp_cv = 0
def nothing(s):
    pass
col_black = (0,0,0,180,255,46)# black
col_red = (0,100,80,10,255,255)# red
col_blue = (100,43,46,124,255,255)# blue
col_green= (35,43,46,77,255,255)# green
col_yellow = (26,43,46,34,255,255)# yellow


Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.mat = None
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.twist = Twist()
        self.tmp = 0
        self.left_fork_confirm_count = 0
        self.left_fork_confirm_need = 3
        self.cx_filtered = None

    def image_callback(self, msg):
        global last_erro
        global tmp_cv
        if self.tmp==0:
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch,'Adjust_hsv',0,4,nothing)
            self.tmp=1
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        kernel = numpy.ones((5,5),numpy.uint8)
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
        m=cv2.getTrackbarPos(Switch,'Adjust_hsv')
        if m == 0:
            lowerbH=col_red[0]
            lowerbS=col_red[1]
            lowerbV=col_red[2]
            upperbH=col_red[3]
            upperbS=col_red[4]
            upperbV=col_red[5]
        elif m == 1:
            lowerbH=col_green[0]
            lowerbS=col_green[1]
            lowerbV=col_green[2]
            upperbH=col_green[3]
            upperbS=col_green[4]
            upperbV=col_green[5]
        elif m == 2:
            lowerbH=col_blue[0]
            lowerbS=col_blue[1]
            lowerbV=col_blue[2]
            upperbH=col_blue[3]
            upperbS=col_blue[4]
            upperbV=col_blue[5]
        elif m == 3:
            lowerbH=col_yellow[0]
            lowerbS=col_yellow[1]
            lowerbV=col_yellow[2]
            upperbH=col_yellow[3]
            upperbS=col_yellow[4]
            upperbV=col_yellow[5]
        elif m == 4:
            lowerbH=col_black[0]
            lowerbS=col_black[1]
            lowerbV=col_black[2]
            upperbH=col_black[3]
            upperbS=col_black[4]
            upperbV=col_black[5]
        else:
            lowerbH=0
            lowerbS=0
            lowerbV=0
            upperbH=255
            upperbS=255
            upperbV=255

        mask=cv2.inRange(hsv_dilate,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        masked = cv2.bitwise_and(image, image, mask=mask)
        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空
        h, w, d = image.shape
        search_top = h-30
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # 计算mask图像的重心，即几何中心
        # 分叉口策略：
        # 1) 若检测到多个连通区域，默认选择最左侧分支
        # 2) 为避免提前转弯，要求分叉连续检测到若干帧后再执行左转分支选择
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        valid_components = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 30:
                valid_components.append((i, area, centroids[i][0], centroids[i][1], stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_HEIGHT]))

        if len(valid_components) >= 2:
            # 是否像“真实分叉”：左右分离足够明显，且分支已进入较近底部区域
            xs = [comp[2] for comp in valid_components]
            min_x = min(xs)
            max_x = max(xs)
            horizontal_gap = max_x - min_x
            near_bottom_components = 0
            for comp in valid_components:
                comp_top = comp[4]
                comp_h = comp[5]
                comp_bottom = comp_top + comp_h
                if comp_bottom >= (h - 10):
                    near_bottom_components += 1
            is_fork_scene = horizontal_gap > (w * 0.20) and near_bottom_components >= 2

            if is_fork_scene:
                self.left_fork_confirm_count += 1
            else:
                self.left_fork_confirm_count = 0

            if self.left_fork_confirm_count >= self.left_fork_confirm_need:
                # 默认左转：选取最左侧分支的重心作为跟踪目标
                target_comp = min(valid_components, key=lambda x: x[2])
                cx = int(target_comp[2])
                cy = int(target_comp[3])
            else:
                # 尚未确认到分叉，先按整体重心走，避免提前切向左支路
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                else:
                    cx = None
                    cy = None
        else:
            self.left_fork_confirm_count = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cx = None
                cy = None

        if cx is not None and cy is not None:
            #cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
            #cv2.circle(image, (cx-60, cy), 10, (0, 0, 255), -1)
            #cv2.circle(image, (w/2, h), 10, (0, 255, 255), -1)
            if cv2.circle:
            # 计算图像中心线和目标指示线中心的距离
                if self.cx_filtered is None:
                    self.cx_filtered = float(cx)
                else:
                    self.cx_filtered = 0.7 * self.cx_filtered + 0.3 * float(cx)

                erro = self.cx_filtered - w/2
                d_erro=erro-last_erro
                self.twist.linear.x = 0.11
                if abs(erro) < 8:
                    self.twist.angular.z = 0.0
                elif erro<0:
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                elif erro>0:
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                else :
                    self.twist.angular.z = 0.0
                last_erro=erro
        else:
            self.cx_filtered = None
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("Adjust_hsv", mask)
        cv2.waitKey(3)
        #cv2.imshow("Adjust_hsv", mask)
        #print('start windows')
        #cv2.waitKey(3)
def main(args=None):
    rclpy.init(args=args)
    print('start patrolling')
    follower = Follower()
    while rclpy.ok():
        rclpy.spin_once(follower)
        time.sleep(0.1)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
