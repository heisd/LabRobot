#!/usr/bin/env python3

import rclpy
import _thread
import threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


angle=[0.0]*3
distan=[0.0]*3

class VisualFollower(Node):
	def __init__(self):
		super().__init__('visualfollower')
		qos = QoSProfile(depth=10)
		
		# 位置看门狗: 超过 position_timeout 秒没收到目标位置(tracker 崩溃/无消息)就停车,
		# 防止目标丢失后小车带着最后一次速度一直跑
		self.position_timeout = 0.5
		self._last_pos_time = None
		self._stopped_by_watchdog = False
		self.watchdog = self.create_timer(0.1, self._watchdog)
		self.switchMode= True  # if this is set to False the O button has to be kept pressed in order for it to move
		self.max_speed = 0.3
		self.controllButtonIndex = -4

		self.buttonCallbackBusy=False
		self.active=False
		self.i=0
		self.cmdVelPublisher = self.create_publisher( Twist,'/cmd_vel', qos)

		# the topic for the messages from the ps3 controller (game pad)

		# the topic for the tracker that gives us the current position of the object we are following
		self.positionSubscriber = self.create_subscription(PositionMsg, '/object_tracker/current_position', self.positionUpdateCallback, qos)
		self.trackerInfoSubscriber = self.create_subscription(StringMsg, '/object_tracker/info', self.trackerInfoCallback, qos)

		# PID parameters first is angular, dist
		targetDist = 600
		#PID_param = rospy.get_param('~PID_controller')
		# the first parameter is the angular target (0 degrees always) the second is the target distance (say 1 meter)
		self.PID_controller = simplePID([0, targetDist], [1.2 ,0.2 ], [0 ,0.00], [0.005 ,0.00])

		# this method gets called when the process is killed with Ctrl+C
		#rclpy.shutdown(self.controllerLoss)
		
	def trackerInfoCallback(self, info):
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		self.get_logger().warn(info.data)
	
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

		# gets called whenever we receive a new position. It will then update the motorcomand
		self._last_pos_time = self.get_clock().now()
		self._stopped_by_watchdog = False

		angleX= position.angle_x
		distance = position.distance


		# call the PID controller to update it and get new speeds
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
		# clip these speeds to be less then the maximal speed specified above
		angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
		linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
		
		# create the Twist message to send to the cmd_vel topic
		velocity = Twist()	

		velocity.linear.x = float(linearSpeed)
		velocity.linear.y = 0.0
		velocity.linear.z = 0.0

		velocity.angular.x = 0.0
		velocity.angular.y = 0.0
		velocity.angular.z = angularSpeed
		if((distance>2000)or((distance==0))):
			self.stopMoving()
			print('out of tracking\n')
		else:
			self.cmdVelPublisher.publish(velocity)
		#self.get_logger().info('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
			

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
		'''Create a discrete PID controller
		each of the parameters may be a vector if they have the same length
		
		Args:
		target (double) -- the target value(s)
		P, I, D (double)-- the PID parameter

		'''

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
		'''Updates the PID controller. 

		Args:
			current_value (double): vector/number of same legth as the target given in the constructor

		Returns:
			controll signal (double): vector of same length as the target

		'''
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.perf_counter()
			return np.zeros(np.size(current_value))
		
		error = self.setPoint - current_value

                #when bias is little, stop moving. errpr[0]=angle(rad),         error[1]=distance(mm)
		#                                  self.setPoint[0]=angle(rad), self.setPoint[1]=distance(mm)
		if error[0]<0.1 and error[0]>-0.1:
			error[0]=0
		if error[1]<150 and error[1]>-150:
			error[1]=0
		
		#when target is little, amplify velocity by amplify error.
		if (error[1]>0 and self.setPoint[1]<1200):
			error[1]=error[1]*(1200/self.setPoint[1])*0.5
			error[0]=error[0]*0.8
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
    print('visualFollower')
    rclpy.init(args=args)
    visualFollower = VisualFollower()
    print('visualFollower init done')
    try:
        rclpy.spin(visualFollower)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前先停车, 再销毁节点 (顺序反了会向已销毁节点发布而报错)
        visualFollower.controllerLoss()
        visualFollower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    


