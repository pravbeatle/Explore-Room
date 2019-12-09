#!/usr/bin/env python

import rospy 
import tf

from smach import State, StateMachine
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from time import sleep
from rospy_message_converter import message_converter
import json
import math
from turtlebot import TurtleBot
from occupancy_grid_map import OccupancyGridMap
import cv2 as cv
import numpy as np

image_capture_path = './src/group27/project/image_capture/'


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


	def execute(self, userdata):
		
		for i in range(150):
			if self.color_found:
				break
			
			bot.move(angular=(0,0, 0.5))
			bot.rate.sleep()
	
		return 'green_found' if self.color_found else 'nothing_found'


class Focus(State):
	def __init__(self, bot):
		State.__init__(self, outcomes=['focus_done'])
		self.object_subscriber = rospy.Subscriber('object_detection/color/json', String, self.process_json, queue_size=1)
		self.closest_radius = 45

	
	
	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		self.object_radius = data['radius']
		self.object_diff = data['angle_from_centroid']
		
	
	def send_bot_status(self):
		status_message = {
							'status': 'focus_done',
							'file_path': image_capture_path
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
		State.__init__(self, outcomes=['nav_done', 'nav_not_done'])
		self.transform_listener = tf.TransformListener()
	
	
	def calculate_pos_quat(self, closest_room):
		center = {'x': closest_room['center_x'], 'y': closest_room['center_y']}
		enterance = {'x': closest_room['enterance_x'], 'y': closest_room['enterance_y']}
		pos_enterance = { 'x': closest_room['enterance_x'], 'y': closest_room['enterance_y'], 'z': self.position[2] }
		pos_center = { 'x': closest_room['center_x'], 'y': closest_room['center_y'], 'z': self.position[2] }
		quat = { 'r1': self.orientation[0], 'r2': self.orientation[1], 'r3': self.orientation[2], 'r4': self.orientation[3] }
		
		closest_room = {
			'center': center,
			'enterance': enterance,
			'pos_center': pos_center,
			'pos_enterance': pos_enterance,
			'quat': quat
		}
		
		return closest_room
		
	
	def dist(self, x, y):
		
		return math.sqrt((x - self.position[0])**2 + (y - self.position[1])**2)
		
	
	def calculate_closest_room(self):
		distances = []
		
		for i in range(len(bot.rooms)):
			bot.rooms[i]['distance_from_bot'] = self.dist(bot.rooms[i]['center_x'], bot.rooms[i]['center_y'])
		
		closest_room = min(bot.rooms, key=lambda x:x['distance_from_bot'])
		
		return closest_room
	
	
	def execute(self, userdata):	
				
		(self.position, self.orientation) = self.transform_listener.lookupTransform("/base_link", "/map", rospy.Time(0))
		
		closest_room = self.calculate_closest_room()
		
		closest_room = self.calculate_pos_quat(closest_room)
		
		bot.set_closest_room(closest_room)
		
		success = bot.go_to_room_enterance()
		
		return 'nav_done' if success else 'nav_not_done'
		


class ExploreRoom(State):
	
	def __init__(self, bot, grid_map):
		State.__init__(self, outcomes=['poster_found', 'poster_not_found'])
		self.object_subscriber = rospy.Subscriber('object_detection/object/json', String, self.process_json, queue_size=1)
		self.poster_found = False
		
	
	
	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		if data['poster_found']:
			self.poster_found = True
		
		
	def send_bot_status(self):
		status_message = {
							'status': 'room_scan'
		}
		status_message = json.dumps(status_message)
		bot.status_publisher.publish(status_message)
	
	
	def circular_scan(self):
		
		for i in range(150):
			if self.poster_found:
				return
			
			bot.move(angular=(0,0, 0.5))
			bot.rate.sleep()
		
	
	
	def execute(self, userdata):
		
		self.send_bot_status()
		
		center_x = bot.closest_room['center']['x']
		center_y = bot.closest_room['center']['x']
		
		wall_points =  grid_map.sample_wall_points(center_x, center_y)
		print('no of pts found: ', len(wall_points))
		
		
		for wall_point in wall_points:
			
			x, y = wall_point['point']
			
			print('going to point : ', (x, y))
			
			success = bot.go_to(x, y, mode='point')
			
			if success:
				wall_point['visited'] = True
				self.circular_scan()
				
			
			if self.poster_found:
				break
		
			
		return 'poster_found' if self.poster_found else 'poster_not_found'		




class FocusPoster(State):
	def __init__(self, bot):
		State.__init__(self, outcomes=['picture_taken'])
		self.object_subscriber = rospy.Subscriber('object_detection/object/json', String, self.process_json, queue_size=1)
		self.closest_poster_area = 45000
		
		# rect area 4k to 45k; w/h ratio 0.5 to 0.7

	
	
	def process_json(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		self.object_faces_found = data['faces_found']
		self.object_poster_diff = data['angle_from_poster']
		self.poster_rectangle_area = data['rect']
		self.poster_found = data['poster_found']
		
		
	
	def send_bot_status(self):
		status_message = {
							'status': 'poster_focus_done',
							'file_path': image_capture_path
		}
		status_message = json.dumps(status_message)
		bot.status_publisher.publish(status_message)	
		
		
	def closeness_constraint(self):
		
		return self.poster_rectangle_area <= self.closest_poster_area
		
	
	def execute(self, userdata):
		
		while self.poster_found and self.closeness_constraint():
			#~ # turn towards the goal
			angular = (0,0,-self.object_poster_diff) 
			#~ # Too far away from object, need to move forwards
			linear = (0.15,0,0)
			bot.move(linear, angular)
			
		self.send_bot_status()
		
		bot.go_to_room_center()
		
		return 'picture_taken'
		
		

###################################################################################


if __name__ == '__main__':
	try: 
		rospy.init_node('Explorer', anonymous=True)
		grid_map = OccupancyGridMap()
		bot = TurtleBot()
	
		sm = StateMachine(outcomes=['success', 'failure'])  # the end states of the machine
		with sm:
			
			StateMachine.add('CSCAN', CScan(bot), transitions={'green_found': 'FOCUS', 'nothing_found': 'CSCAN'})
			StateMachine.add('FOCUS', Focus(bot), transitions={'focus_done': 'NAV_TO_ROOM'})
			StateMachine.add('NAV_TO_ROOM', NavRoom(bot), transitions={'nav_done': 'EXPLORE_ROOM', 'nav_not_done': 'CSCAN'})
			StateMachine.add('EXPLORE_ROOM', ExploreRoom(bot, grid_map), transitions={'poster_found': 'FOCUS_ON_POSTER', 'poster_not_found': 'failure'})
			StateMachine.add('FOCUS_ON_POSTER', FocusPoster(bot), transitions={'picture_taken': 'success'})
			
			
			sm.execute()
		
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo('Quitting...')


