#!/usr/bin/env python


import rospy 
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

import tf

from smach import State, StateMachine
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from time import sleep
from rospy_message_converter import message_converter
import json
import yaml
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
		
		if data['circle_found'] and data['object_color'] == 'Green':
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
		self.closest_radius = 30

	
	
	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		self.object_radius = data['radius']
		self.object_diff = data['angle_from_centroid']
		
	
	def send_bot_status(self):
		status_message = {
							'status': 'focus_done',
							'file_path': './src/group27/project/image_capture/green_circle.jpg'
		}
		status_message = json.dumps(status_message)
		bot.status_publisher.publish(status_message)
		
	
	def execute(self, userdata):
		
		while self.object_radius <= self.closest_radius:
			# turn towards the goal
			angular = (0,0,-self.object_diff) 
			# Too far away from object, need to move forwards
			linear = (0.2,0,0)
			bot.move(linear, angular)
		
		self.send_bot_status()
							
		return 'focus_done'
		


class NavRoom(State):
	def __init__(self, bot):
		State.__init__(self, outcomes=['nav_done'])
		#~ self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.tf_listener = tf.TransformListener()
	
	
	#~ def odom_callback(self, data):
		#~ self.position = data.pose.pose.position
		#~ self.orientation = data.pose.pose.orientation
	
	
	def create_pose_quat(self, x, y):
		pos = { 'x': x, 'y': y, 'z': self.position[2] }
		quat = { 'r1': self.orientation[0], 'r2': self.orientation[1], 'r3': self.orientation[2], 'r4': self.orientation[3] }
		
		return pos, quat
		
	
	def dist(self, x, y):
		
		return math.sqrt((x - self.position[0])**2 + (y - self.position[1])**2)
		
	
	def calculate_closest_room(self):
		distances = []
		
		for room in bot.rooms:
			distances.append( self.dist(room['center_x'], room['center_y']) )
		
		closest_room_index = distances.index(min(distances))
		closest_room = bot.rooms[closest_room_index]
		
		return closest_room
	
	
	def send_nav_goals(self, closest_room):
		# go to enterance center
		pos, quat = self.create_pose_quat(closest_room['enterance_x'], closest_room['enterance_y'])
		bot.go_to(pos, quat)
		
		# go to room center
		pos, quat = self.create_pose_quat(closest_room['center_x'], closest_room['center_y'])
		bot.go_to(pos, quat)
		
	
	
	def execute(self, userdata):
				
		(self.position, self.orientation) = self.tf_listener.lookupTransform('/odom', '/map', rospy.Time(0))
		
		closest_room = self.calculate_closest_room()
		
		self.send_nav_goals(closest_room)
		
		return 'nav_done'
		


class RoomScan(State):
	def __init__(self, bot):
		State.__init__(self, outcomes=['scan_done'])
		self.object_subscriber = rospy.Subscriber('object_detection/object/json', String, self.process_json, queue_size=1)
		
	
	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		print(data)
		
		
	def send_bot_status(self):
		status_message = {
							'status': 'room_scan'
		}
		status_message = json.dumps(status_message)
		bot.status_publisher.publish(status_message)
		
	
	def execute(self, userdata):
		
		self.send_bot_status()
		
		while not rospy.is_shutdown():
			bot.move(angular=(0,0, math.pi/18))
		
		return 'scan_done'
		
		

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
		
		self.status_publisher = rospy.Publisher('explorer_bot/status', String, queue_size=1)
		
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait until the action server comes up")
		
		self.move_base.wait_for_server(rospy.Duration(5))
		
		with open('./src/group27/project/example/input_points.yaml', 'r') as stream:
			try:
				pts = yaml.safe_load(stream)
				self.rooms = [
					{'center_x': pts['room1_centre_xy'][0], 'center_y': pts['room1_centre_xy'][1], 'enterance_x': pts['room1_entrance_xy'][0], 'enterance_y': pts['room1_entrance_xy'][1]},
					{'center_x': pts['room2_centre_xy'][0], 'center_y': pts['room2_centre_xy'][1], 'enterance_x': pts['room2_entrance_xy'][0], 'enterance_y': pts['room2_entrance_xy'][1]}
				]
			except yaml.YAMLError as e:
				print(e)
		
	
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
			StateMachine.add('FOCUS', Focus(bot), transitions={'focus_done': 'NAV_TO_ROOM'})
			StateMachine.add('NAV_TO_ROOM', NavRoom(bot), transitions={'nav_done': 'ROOM_SCAN'})
			StateMachine.add('ROOM_SCAN', RoomScan(bot), transitions={'scan_done': 'success'})
			#~ StateMachine.add('FOCUS_ON_POSTER', FocusPoster(bot), transitions={'poster_done': 'success'})
			
			sm.execute()
	

	except rospy.ROSInterruptException:
		rospy.loginfo('Quitting...')


