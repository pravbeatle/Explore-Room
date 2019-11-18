#!/usr/bin/env python


import rospy 
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from smach import State, StateMachine
from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep
from rospy_message_converter import message_converter
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import json
import math

############################## STATE MACHINE ####################################


class CScan(State):
	def __init__(self, bot):
		State.__init__(self, outcomes=['green_found', 'nothing_found'])
		self.object_subscriber = rospy.Subscriber('object_detection/color/json', String, self.process_json, queue_size=1)

		self.color_found = False


	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		if data['circle_found']:
			self.color_found = True
	
	
	def color_or_scan_done(self, rad):
		
		if rad >= 2*math.pi or self.color_found:
			return True
		
		return False


	def execute(self, userdata):

		# EDIT REQUIRED :- change code to only do one 360deg scan
		radians_done = 0
		
		while not rospy.is_shutdown() and not self.color_or_scan_done(radians_done):
			bot.move(angular=(0,0, math.pi/18))
	
	
		return 'green_found'


class Focus(State):
	def __init__(self, bot):
		State.__init__(self, outcomes=['focus_done'])
		self.object_subscriber = rospy.Subscriber('object_detection/color/json', String, self.process_json, queue_size=1)
		self.camera_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
		self.cv_bridge = CvBridge()
		
	
	def image_callback(self, data):
		try:
			self.object_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8") 
		except CvBridgeError as e:
			print(e)
	
	
	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		self.object_radius = data['radius']
		self.object_diff = data['angle_from_centroid']
		
	
	def execute(self, userdata):
		
		while self.object_radius <= 30:
			# turn towards the goal
			angular = (0,0,-self.object_diff) 
			# Too far away from object, need to move forwards
			linear = (0.2,0,0)
			bot.move(linear, angular)
		
		cv.imwrite('./project/image_capture/green_circle.png', self.object_image)
			
		return 'focus_done'

###################################################################################

class TurtleBot:
	def __init__(self):
	
		self.goal_sent = False	# if goal is sent, we need to cancel before shutting down
		
		rospy.on_shutdown(self.shutdown)
		
		self.rate = rospy.Rate(10)
		
		# set linear and angular velocities
		self.velocity = Twist()
		
		# publish and subcribe to relevant topics
		self.velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait until the action server comes up")
		
		self.move_base.wait_for_server(rospy.Duration(5))
		
	
	def move(self, linear=(0,0,0), angular=(0,0,0)):
	
		self.velocity.linear.x = linear[0] 	# Forward or Backward with in m/sec.
		self.velocity.linear.y = linear[1]
		self.velocity.linear.z = linear[2]
		
		self.velocity.angular.x = angular[0]
		self.velocity.angular.y = angular[1]
		self.velocity.angular.z = angular[2] 	# Anti-clockwise/clockwise in radians per sec
		
		self.velocity_publisher.publish(self.velocity)
		
	
	def go_to(self, pose, quat):
	
		# send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pose['x'], pose['y'], pose['z']), 
		Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
		
		self.move_base.send_goal(goal)
		
		rospy.loginfo('Sent goal and waiting for robot to carry it out...')
		success = self.move_base.wait_for_result(rospy.Duration(60))
		
		state = self.move_base.get_state()
		result = False
		
		if success and state == GoalStatus.SUCCEEDED:
			result = True
		else:
			self.move_base.cancel_goal()
		
			self.goal_sent = False
		
		return result 
		
	
	def shutdown(self):
		if self.goal_sent:
			self.move_base.cancel_goal()
			rospy.loginfo("Stop Robot")
			rospy.sleep(1)
	

if __name__ == '__main__':
	try: 
		rospy.init_node('Explorer', anonymous=True)
		bot = TurtleBot()
		
		sm = StateMachine(outcomes=['success'])  # the end states of the machine
		with sm:
			StateMachine.add('CSCAN', CScan(bot), transitions={'green_found': 'FOCUS', 'nothing_found': 'CSCAN'})
			StateMachine.add('FOCUS', Focus(bot), transitions={'focus_done': 'success'})
			#~ StateMachine.add('NAV_TO_ROOM', NavRoom(bot), transitions={'nav_done': 'ROOM_SCAN'})
			#~ StateMachine.add('ROOM_SCAN', RoomScan(bot), transitions={'scan_done': 'FOCUS_ON_POSTER'})
			#~ StateMachine.add('FOCUS_ON_POSTER', FocusPoster(bot), transitions={'poster_done': 'success'})
			
			sm.execute()
			
			
			x = -2.3
			y = 5.63
			theta = 0.1
			
			position = {'x': x, 'y': y, 'z': 0.0}
			quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta/2.0), 'r4': np.cos(theta/2.0)}
			
			#success = navigator.go_to(position, quaternion)
			
			#if success: 
			#	rospy.loginfo('reached goal')
			#else:
			#	rospy.loginfo('failed to reach the desired goal')
			
			#~ rospy.spin()
	

	except rospy.ROSInterruptException:
		rospy.loginfo('Quitting...')






