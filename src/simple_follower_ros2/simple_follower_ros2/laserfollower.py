#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
import _thread
import threading
import time
import numpy as np 
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import Int8

import rclpy
from rclpy.node import Node
angle=[0.0]*3
distan=[0.0]*3

class LaserFollower(Node):

	def __init__(self):
		super().__init__('laserfollower')
		# 位置看门狗: 目标丢失(tracker 停发位置)后超时停车, 防止小车带着最后速度一直跑
		self.position_timeout = 0.5
		self._last_pos_time = None
		self._stopped_by_watchdog = False
		self.watchdog = self.create_timer(0.1, self._watchdog)
		# PID 增益: 第一分量为角度, 第二分量为距离; 带默认值, 无参数文件也能正常启动
		self.declare_parameter('P', [1.6, 0.5])
		self.declare_parameter('I', [0.0, 0.0])
		self.declare_parameter('D', [0.03, 0.005])
		self.declare_parameter('targetDist', 0.8)
		self.declare_parameter('maxSpeed', 0.4)
		# as soon as we stop receiving Joy messages from the ps3 controller we stop all movement:
		self.switchMode= True
		self.max_speed = self.get_parameter('maxSpeed').get_parameter_value().double_value
		#self.controllButtonIndex = self.declare_parameter('~controllButtonIndex').value
		self.controllButtonIndex = -4
		self.buttonCallbackBusy=False
		self.active=False
		self.i=0
		qos = QoSProfile(depth=10)
		self.laserfwflagPublisher = self.create_publisher(Int8,'/laser_follow_flag',qos)
		self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', qos)
		self.positionSubscriber = self.create_subscription(
		    PositionMsg,
		    '/object_tracker/current_position',
		    self.positionUpdateCallback,
		    qos)
		self.trackerInfoSubscriber = self.create_subscription(
		    StringMsg,
		    '/object_tracker/info',
		    self.trackerInfoCallback,
		    qos)
		targetDist = self.get_parameter('targetDist').get_parameter_value().double_value
		P = list(self.get_parameter('P').get_parameter_value().double_array_value)
		I = list(self.get_parameter('I').get_parameter_value().double_array_value)
		D = list(self.get_parameter('D').get_parameter_value().double_array_value)
		self.PID_controller = simplePID([0.0, targetDist], P, I, D)
		# PID parameters first is angular, dist
	def trackerInfoCallback(self, info):
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		self.get_logger().warn(info.data)
	def publish_flag(self):
		laser_follow_flag=Int8()
		laser_follow_flag.data=1
		self.laserfwflagPublisher.publish(laser_follow_flag)
		self.get_logger().info('laser_follow_flag: {}'.format(laser_follow_flag))
	def _watchdog(self):
		# 收到过位置后, 若 position_timeout 内不再有新位置, 判定为失联并停车
		if self._last_pos_time is None:
			return
		dt = (self.get_clock().now() - self._last_pos_time).nanoseconds * 1e-9
		if dt > self.position_timeout:
			if not self._stopped_by_watchdog:
				self.get_logger().warn('position timeout, stop moving')
				self._stopped_by_watchdog = True
			self.stopMoving()
	def positionUpdateCallback(self, position):
		self._last_pos_time = self.get_clock().now()
		self._stopped_by_watchdog = False
		angle_x= position.angle_x
		distance = position.distance
		#print(distance)
		if(angle_x>0):
			angle_x=angle_x-3.1415
		else :
			angle_x=angle_x+3.1415

		# call the PID controller to update it and get new speeds
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle_x, distance])
		# clip these speeds to be less then the maximal speed specified above
		angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
		linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
		# create the Twist message to send to the cmd_vel topic
		velocity = Twist()	
		velocity.linear.y = 0.0
		velocity.linear.z = 0.0
		velocity.angular.x = 0.0
		velocity.angular.y = 0.0
		if distance>0.15:
			velocity.linear.x = linearSpeed
			velocity.angular.z = -angularSpeed
		else:
			velocity.linear.x = 0.0
			velocity.angular.z = 0.0
		#self.get_logger().info('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
		self.cmdVelPublisher.publish(velocity)
		self.publish_flag()
	def stopMoving(self):
		velocity = Twist()
		velocity.linear.x = 0.0
		velocity.linear.y = 0.0
		velocity.linear.z = 0.0

		velocity.angular.x = 0.0
		velocity.angular.y = 0.0
		velocity.angular.z = 0.0
		self.cmdVelPublisher.publish(velocity)
	def controllerLoss(self):
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()
		self.active = False
		self.get_logger().info('lost connection')
class simplePID:
	'''very simple discrete PID controller'''
	def __init__(self, target, P, I, D):
		# check if parameter shapes are compatabile.
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 
	def update(self, current_value):

		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.perf_counter()
			return np.zeros(np.size(current_value))

		
		error = self.setPoint - current_value

		if error[0]<0.1 and error[0]>-0.1:
			error[0]=0
		if error[1]<0.1 and error[1]>-0.1:
			error[1]=0
		
		#when target is little, amplify velocity by amplify error.
		if (error[1]>0 and self.setPoint[1]<1.2):
			error[1]=error[1]*(1.2/self.setPoint[1])*0.7

		P =  error
		
		currentTime = time.perf_counter()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		D = (error-self.last_error)/deltaT
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		return self.Kp*P + self.Ki*I + self.Kd*D
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    laserfollower = LaserFollower()
    try:
        rclpy.spin(laserfollower)
    finally:
        laserfollower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

