#!/usr/bin/env python3
from __future__ import division
import message_filters
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt
import cv_bridge

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

np.seterr(divide='ignore', invalid='ignore')
displayImage=False
plt.close('all')

class VisualTracker(Node):
	def __init__(self):
		# 调用父类的构造函数
		super().__init__('visualtracker')
		# 创建一个 QoS 配置文件
		qos = QoSProfile(depth=10)
		# 初始化 OpenCV 的桥接器
		self.bridge = cv_bridge.CvBridge()
		#self.tmp_list = self.get_parameter('~targetred/upper').value
		# 红色在 HSV 中跨越 0/180 两端, 需要两段范围相或才能完整覆盖;
		# S/V 取较高下限以滤除低饱和/暗背景 (参数顺序: 下限, 上限)
		self.redLower1 = np.array([0, 100, 80])
		self.redUpper1 = np.array([10, 255, 255])
		self.redLower2 = np.array([160, 100, 80])
		self.redUpper2 = np.array([180, 255, 255])
		
		#self.pictureHeight= self.get_parameter('~pictureDimensions/pictureHeight')
		#self.pictureWidth = self.get_parameter('~pictureDimensions/pictureWidth')
		#vertAngle =self.get_parameter('~pictureDimensions/verticalAngle')
		#horizontalAngle =  self.get_parameter('~pictureDimensions/horizontalAngle')
		# 图片的大小
		self.pictureHeight= 400
		self.pictureWidth = 640
		# 摄像头的视角
		vertAngle =0.43196898986859655
		# 水平视角
		horizontalAngle =  0.5235987755982988
		
		# precompute tangens since thats all we need anyways:
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)	
		self.lastPosition = None
		# one callback that deals with depth and rgb at the same time
		im_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
		dep_sub = message_filters.Subscriber(self,Image,'/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
		queue_size = 30
 
		self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub],queue_size,0.05)
		self.timeSynchronizer.registerCallback(self.trackObject)
		self.positionPublisher = self.create_publisher( PositionMsg,'/object_tracker/current_position', qos)
		self.posMsg=PositionMsg()

	# 检测和跟踪物体
	def trackObject(self, image_data, depth_data):
		if(image_data.encoding != 'rgb8'):
			raise ValueError('image is not rgb8 as expected')
		#convert both images to numpy arrays
		# 转换为 OpenCV 图像
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
		depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1")	
		# 按实际图像尺寸更新, 避免分辨率与硬编码不一致时直接抛异常
		self.pictureHeight, self.pictureWidth = frame.shape[:2]
		# blure a little and convert to HSV color space
		#blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)	
		# select all the pixels that are in the range specified by the target
		org_mask = cv2.bitwise_or(
			cv2.inRange(hsv, self.redLower1, self.redUpper1),
			cv2.inRange(hsv, self.redLower2, self.redUpper2))

		# clean that up a little, the iterations are pretty much arbitrary
		mask = cv2.erode(org_mask, None, iterations=4)		
		#self.get_logger().warn('no position found')
		#mask = cv2.dilate(mask,None, iterations=3)
		# find contours of the object
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

		# lets you display the image for debuging. Not in realtime though
		if displayImage:
			backConverted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
			#cv2.imshow('frame', backConverted)
			#cv2.waitKey(0)
			#print(backConverted)			
			plt.figure()
			plt.subplot(2,2,1)
			plt.imshow(frame)
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,2)
			plt.imshow(org_mask, cmap='gray', interpolation = 'bicubic')			
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,3)			
			plt.imshow(mask, cmap='gray', interpolation = 'bicubic')
			plt.xticks([]),plt.yticks([])
			plt.show()
			rclpy.sleep(0.2)
		# 选取面积最大的轮廓作为目标(最稳健的近似)
		ordered = sorted(contours, key=cv2.contourArea, reverse=True)
		if len(ordered) == 0:
			# 完全没有检测到目标 -> 发布 distance=0, 让下游 follower 立即停车
			self.lastPosition = None
			self.publishLost()
			return
		pos = self.analyseContour(ordered[0], depthFrame)
		self.lastPosition = pos
		self.publishPosition(pos)
		
	def publishPosition(self, pos):
		# calculate the angles from the raw position
		self.posMsg.angle_x = self.calculateAngleX(pos)
		print(self.posMsg.angle_x)
		self.posMsg.angle_y = self.calculateAngleY(pos)
		self.posMsg.distance=float(pos[1])
		# publish the position (angleX, angleY, distance)

		self.positionPublisher.publish(self.posMsg)

	def publishLost(self):
		'''目标丢失: 发布 distance=0 (以及 0 角度), 通知下游 follower 停车.'''
		self.posMsg.angle_x = 0.0
		self.posMsg.angle_y = 0.0
		self.posMsg.distance = 0.0
		self.positionPublisher.publish(self.posMsg)

	def checkPosPlausible(self, pos):
		'''Checks if a position is plausible. i.e. close enough to the last one.'''

		# for the first scan we cant tell
		if self.lastPosition is None:
			return False

		# unpack positions
		((centerX, centerY), dist)=pos	
		((PcenterX, PcenterY), Pdist)=self.lastPosition
		
		if np.isnan(dist):
			return False

		# distance changed to much
		if abs(dist-Pdist)>0.5:
			return False

		# location changed to much (5 is arbitrary)
		if abs(centerX-PcenterX)>(self.pictureWidth /5):
			return False

		if abs(centerY-PcenterY)>(self.pictureHeight/5):
			return False
		
		return True
			
		
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''
		centerX = pos[0][0]
		displacement = 2*centerX/self.pictureWidth-1
		angle = -1*np.arctan(displacement*self.tanHorizontal)
		return angle

	def calculateAngleY(self, pos):
		centerY = pos[0][1]
		displacement = 2*centerY/self.pictureHeight-1
		angle = -1*np.arctan(displacement*self.tanVertical)
		return angle
	
	def analyseContour(self, contour, depthFrame):
		'''Calculates the centers coordinates and distance for a given contour

		Args:
			contour (opencv contour): contour of the object
			depthFrame (numpy array): the depth image
		
		Returns:
			centerX, centerY (doubles): center coordinates
			averageDistance : distance of the object
		'''
		# get a rectangle that completely contains the object
		centerRaw, size, rotation = cv2.minAreaRect(contour)

		# get the center of that rounded to ints (so we can index the image)
		center = np.round(centerRaw).astype(int)

		# find out how far we can go in x/y direction without leaving the object (min of the extension of the bounding rectangle/2 (here 3 for safety)) 
		minSize = int(min(size)/3)

		# get all the depth points within this area (that is within the object)
		# 裁剪到图像边界内, 防止负索引回绕取到错误区域
		y0 = max(0, center[1] - minSize)
		y1 = min(depthFrame.shape[0], center[1] + minSize)
		x0 = max(0, center[0] - minSize)
		x1 = min(depthFrame.shape[1], center[0] + minSize)
		depthObject = depthFrame[y0:y1, x0:x1]

		# get the average of all valid points (average to have a more reliable distance measure)
		# 先转 float 再过滤: 深度可能是 16UC1(整型, np.isnan 会报错); 0 表示无效读数也一并剔除
		depthObject = depthObject.astype(np.float32)
		depthArray = depthObject[np.isfinite(depthObject) & (depthObject > 0)]
		#averageDistance = np.mean(depthArray)
		
		if len(depthArray) == 0:
			self.get_logger().warn('empty depth array. all depth values are nan')
			averageDistance=0
		else:
			averageDistance = np.mean(depthArray)
			
		# 有效深度区间 (mm); 超出该范围视为无效, 置 0 交由上层判定为"丢失目标"
		if averageDistance < 400 or averageDistance > 3000:
			averageDistance = 0.0


		return (centerRaw, averageDistance)
	
def main(args=None):
    print('visual_tracker')
    rclpy.init(args=args)
    visualTracker = VisualTracker()
    
    print('visualTracker init done')
    try:
        rclpy.spin(visualTracker)
    finally:
        visualTracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    


