#!/usr/bin/env python


import rospy 
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion


class TurtleBot:
	def __init__(self):
		
		self.goal_sent = False	# if goal is sent, we need to cancel before shutting down
		
		rospy.on_shutdown(self.shutdown)
		
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait until the action server comes up")
		
		self.move_base.wait_for_server(rospy.Duration(5))
		
	
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
		navigator = TurtleBot()
		
		x = -2.3
		y = 5.63
		theta = 0.1
		
		position = {'x': x, 'y': y, 'z': 0.0}
		quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta/2.0), 'r4': np.cos(theta/2.0)}
		
		success = navigator.go_to(position, quaternion)
		
		if success: 
			rospy.loginfo('reached goal')
		else:
			rospy.loginfo('failed to reach the desired goal')
		
		rospy.spin()
		
	
	except rospy.ROSInterruptException:
		rospy.loginfo('Quitting...')
		
			




