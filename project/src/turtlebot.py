import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import String

import yaml
import numpy as np

input_points_path = './src/group27/project/example/input_points.yaml'

class TurtleBot:
	def __init__(self):
	
		self.goal_sent = False	# if goal is sent, we need to cancel before shutting down
		
		rospy.on_shutdown(self.shutdown)
		
		self.rate = rospy.Rate(10)
		
		# set linear and angular velocities
		self.velocity = Twist()
		
		self.closest_room = None
		
		# publish and subcribe to relevant topics
		self.velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		
		self.status_publisher = rospy.Publisher('explorer_bot/status', String, queue_size=1)
		
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait until the action server comes up")
		
		self.move_base.wait_for_server(rospy.Duration(5))
		
		with open(input_points_path, 'r') as stream:
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
		
	
	def calculate_pos_quat(self, x, y):
		theta = 0.1
		pos = {'x': x, 'y' : y, 'z': 0}
		quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
		
		return pos, quat
	
	
	def go_to(self, pose, quat, mode=''):
		
		if mode == 'point':
			pose, quat = self.calculate_pos_quat(pose, quat)
	
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
		
	
	def set_closest_room(self, closest_room):
		self.closest_room = closest_room
		
	def go_to_room_center(self):
		result = self.go_to(self.closest_room['pos_center'], self.closest_room['quat'])
		
		return result
	
	def go_to_room_enterance(self):
		result = self.go_to(self.closest_room['pos_enterance'], self.closest_room['quat'])
		
		return result
		
	
	def shutdown(self):
		if self.goal_sent:
			self.move_base.cancel_goal()
			rospy.loginfo("Stop Robot")
			rospy.sleep(1)




