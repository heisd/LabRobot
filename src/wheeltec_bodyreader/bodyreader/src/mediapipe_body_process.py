#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from bodyreader_msg.msg import Bodylist, Body, Bodyposture
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import Twist
import math

class BodyDataProcess(Node):
    def __init__(self):
        super().__init__('body_process')
        
        self.declare_parameter('open_switch', False)
        self.open_switch = self.get_parameter('open_switch').value
        
        self.bodyposture_pub = self.create_publisher(Bodyposture, "/body_posture", 1)
        self.mode_pub = self.create_publisher(Int8, "/mode", 1)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        
        self.bodylist_sub = self.create_subscription(Bodylist, "/bodylist", self.bodylist_callback, 1)
        self.recoveryid_sub = self.create_subscription(Int16, "/recoveryid", self.recoveryid_callback, 1)
        
        self.lock_body_id = 0
        self.lock_status = 0 # 0: nobody, 1: no lock, 2: locked
        self.last_lock_status = 0
        self.mode_data = 1
        
        # Joint definitions (matches C++ defines)
        self.HEAD = 0
        self.SHOULDER_SPINE = 1
        self.LEFT_SHOULDER = 2
        self.LEFT_ELBOW = 3
        self.LEFT_HAND = 4
        self.RIGHT_SHOULDER = 5
        self.RIGHT_ELBOW = 6
        self.RIGHT_HAND = 7
        self.MID_SPINE = 8
        self.BASE_SPINE = 9
        self.LEFT_HIP = 10
        self.LEFT_KNEE = 11
        self.LEFT_FOOT = 12
        self.RIGHT_HIP = 13
        self.RIGHT_KNEE = 14
        self.RIGHT_FOOT = 15
        self.LEFT_WRIST = 16
        self.RIGHT_WRIST = 17
        self.NECK = 18

        self.get_logger().info("MediaPipe Body Process initialized")

    def recoveryid_callback(self, msg):
        self.lock_body_id = msg.data
        # tips logic if needed

    def judge_pose(self, body, posture_msg):
        # MediaPipe coords might need scaling or different thresholds than Astra
        # Original Astra world coords were in mm. MediaPipe is normalized or m.
        # However, follower.cpp expect mm-like scale for distance (centerofmass.z)
        
        wp = [j.worldposition for j in body.joints]
        
        # Simple pose detection logic (mirrored from C++)
        # Using relative distances
        
        # Left Arm Out
        if (abs(wp[self.LEFT_SHOULDER].y - wp[self.LEFT_ELBOW].y) < 100 
            and abs(wp[self.LEFT_ELBOW].y - wp[self.LEFT_HAND].y) < 150
            and (wp[self.LEFT_SHOULDER].x - wp[self.LEFT_ELBOW].x) > 200):
            posture_msg.left_arm_out = 1
            
        # Right Arm Out
        if (abs(wp[self.RIGHT_SHOULDER].y - wp[self.RIGHT_ELBOW].y) < 100 
            and abs(wp[self.RIGHT_ELBOW].y - wp[self.RIGHT_HAND].y) < 150
            and (wp[self.RIGHT_ELBOW].x - wp[self.RIGHT_SHOULDER].x) > 200):
            posture_msg.right_arm_out = 1

        # Left Hand Raised
        if (wp[self.LEFT_HAND].y - wp[self.LEFT_ELBOW].y) > 180:
            posture_msg.left_hand_raised = 1

        # Akimibo (Lock body)
        if ((wp[self.LEFT_HAND].y - wp[self.BASE_SPINE].y) > 30 
            and (wp[self.RIGHT_HAND].y - wp[self.BASE_SPINE].y) > 30):
            if self.lock_body_id == 0:
                self.lock_body_id = body.bodyid
            posture_msg.akimibo = 1

    def bodylist_callback(self, msg):
        posture_msg = Bodyposture()
        
        if msg.count != 0:
            self.lock_status = 1
        else:
            self.lock_status = 0
            self.cmd_vel_pub.publish(Twist())
            
        for i in range(msg.count):
            body = msg.bodies[i]
            self.judge_pose(body, posture_msg)
            
            if body.bodyid == self.lock_body_id or self.lock_body_id == 0:
                if self.lock_body_id == 0: self.lock_body_id = body.bodyid
                
                posture_msg.bodyid = body.bodyid
                posture_msg.centerofmass_x = body.centerofmass.x
                posture_msg.centerofmass_y = body.centerofmass.y
                posture_msg.centerofmass_z = body.centerofmass.z
                self.lock_status = 2

        posture_msg.lock_status = self.lock_status
        self.bodyposture_pub.publish(posture_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BodyDataProcess()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
