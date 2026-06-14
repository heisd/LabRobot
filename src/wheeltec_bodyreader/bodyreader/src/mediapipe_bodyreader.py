#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bodyreader_msg.msg import Bodylist, Body, Joint, Vector2f, Vector3f
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np

class MediaPipeBodyReader(Node):
    def __init__(self):
        super().__init__('body_main')
        self.bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.522
        )

        self.bodylist_pub = self.create_publisher(Bodylist, "/bodylist", 1)
        self.image_pub = self.create_publisher(Image, "/image_raw", 1)

        self.color_sub = self.create_subscription(Image, "/camera/color/image_raw", self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_raw", self.depth_callback, 10)

        self.latest_depth = None
        
        # Camera Intrinsics (Astra standard, can be adjusted)
        self.fx = 580.0
        self.fy = 580.0
        self.cx = 320.0
        self.cy = 240.0

        self.get_logger().info("MediaPipe BodyReader initialized (x64 replacement)")

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width, _ = cv_image.shape

        # MediaPipe RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_image)

        bodylist_msg = Bodylist()
        bodylist_msg.count = 0

        if results.pose_landmarks:
            bodylist_msg.count = 1
            body = Body()
            body.bodyid = 1
            
            landmarks = results.pose_landmarks.landmark
            
            # Joint Mapping (0-18)
            # 0:HEAD, 1:SHOULDER_SPINE, 2:L_SHOULDER, 3:L_ELBOW, 4:L_HAND, 5:R_SHOULDER, 6:R_ELBOW, 7:R_HAND, 
            # 8:MID_SPINE, 9:BASE_SPINE, 10:L_HIP, 11:L_KNEE, 12:L_FOOT, 13:R_HIP, 14:R_KNEE, 15:R_FOOT, 
            # 16:L_WRIST, 17:R_WRIST, 18:NECK
            
            def get_joint(idx_mp):
                j = Joint()
                lm = landmarks[idx_mp]
                px = int(lm.x * image_width)
                py = int(lm.y * image_height)
                
                j.depthposition.x = float(px)
                j.depthposition.y = float(py)
                
                depth_val = 0.0
                if self.latest_depth is not None:
                    # Basic depth sampling
                    if 0 <= px < image_width and 0 <= py < image_height:
                        depth_val = float(self.latest_depth[py, px])
                
                if depth_val > 0:
                    # 2D to 3D projection
                    j.worldposition.z = depth_val
                    j.worldposition.x = (px - self.cx) * depth_val / self.fx
                    j.worldposition.y = (py - self.cy) * depth_val / self.fy
                else:
                    j.worldposition.x = 0.0
                    j.worldposition.y = 0.0
                    j.worldposition.z = 0.0
                return j

            # Map existing landmarks
            mapped_joints = {
                0: 0,   # Head
                2: 11,  # L_Shoulder
                3: 13,  # L_Elbow
                4: 15,  # L_Hand
                5: 12,  # R_Shoulder
                6: 14,  # R_Elbow
                7: 16,  # R_Hand
                10: 23, # L_Hip
                11: 25, # L_Knee
                12: 27, # L_Foot
                13: 24, # R_Hip
                14: 26, # R_Knee
                15: 28, # R_Foot
                16: 15, # L_Wrist -> Hand
                17: 16, # R_Wrist -> Hand
            }

            for i in range(19):
                body.joints[i] = Joint()
                body.joints[i].type = i

            for target, source in mapped_joints.items():
                body.joints[target] = get_joint(source)
                body.joints[target].type = target

            # Interpolations
            def interpolate(j1, j2, weight=0.5):
                res = Joint()
                res.depthposition.x = j1.depthposition.x * weight + j2.depthposition.y * (1-weight)
                res.depthposition.y = j1.depthposition.y * weight + j2.depthposition.y * (1-weight)
                res.worldposition.x = j1.worldposition.x * weight + j2.worldposition.x * (1-weight)
                res.worldposition.y = j1.worldposition.y * weight + j2.worldposition.y * (1-weight)
                res.worldposition.z = j1.worldposition.z * weight + j2.worldposition.z * (1-weight)
                return res

            # 1: SHOULDER_SPINE (Midpoint of shoulders)
            body.joints[1] = interpolate(body.joints[2], body.joints[5])
            body.joints[1].type = 1
            
            # 18: NECK (Between head and shoulder spine)
            body.joints[18] = interpolate(body.joints[0], body.joints[1])
            body.joints[18].type = 18
            
            # 9: BASE_SPINE (Midpoint of hips)
            body.joints[9] = interpolate(body.joints[10], body.joints[13])
            body.joints[9].type = 9
            
            # 8: MID_SPINE (Between shoulder spine and base spine)
            body.joints[8] = interpolate(body.joints[1], body.joints[9])
            body.joints[8].type = 8

            body.centerofmass.x = body.joints[8].worldposition.x
            body.centerofmass.y = body.joints[8].worldposition.y
            body.centerofmass.z = body.joints[8].worldposition.z
            
            bodylist_msg.bodies[0] = body

        self.bodylist_pub.publish(bodylist_msg)
        
        # Publish image_raw as well (resized like original)
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeBodyReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
