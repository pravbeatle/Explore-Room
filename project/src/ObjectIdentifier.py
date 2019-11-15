#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2 as cv
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

	def __init__(self):
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session
		# Initialise any flags that signal a colour has been detected in view
		self.color_search = {'Red': False, 'Green': False}

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.color_sensitivity = 15
		
		self.image = None	# the image that we continuisly process on top of


		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.cv_bridge = CvBridge()
		
		self.camera_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
		# We covered which topic to subscribe to should you wish to receive image data
		
		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
		self.rate = rospy.Rate(10) 		#10hz
		
		# publish and subcribe to relevant topics
		self.velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		self.color_publisher = rospy.Publisher('object', String, queue_size=10)
				
	
	def extract_colors(self, cv_image):
		# Set the upper and lower bounds for the two colours you wish to identify
		hsv_green_lower = np.array([60 - self.color_sensitivity, 100, 100])
		hsv_green_upper = np.array([60 + self.color_sensitivity, 255, 255])
		# Red has lower and upper bounds split on eithr side
		hsv_red_lower0 = np.array([0, 100, 100])
		hsv_red_upper0 = np.array([self.color_sensitivity, 255, 255])
		hsv_red_lower1 = np.array([180 - self.color_sensitivity, 100, 100])
		hsv_red_upper1 = np.array([180, 255, 255])
		
		hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
		# Filter out everything but particular colours using the cv2.inRange() method
		green_mask = cv.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
		red_mask0 = cv.inRange(hsv_image, hsv_red_lower0, hsv_red_upper0)
		red_mask1 = cv.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
		red_mask = red_mask0 + red_mask1
		# To combine the masks you should use the cv2.bitwise_or() method
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours
		combined_masks = cv.bitwise_or(green_mask, red_mask)
		# Apply the mask to the original image using the cv2.bitwise_and() method
		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image. 
		self.image = cv.bitwise_and(cv_image, cv_image, mask=combined_masks)
		
		return red_mask, green_mask
		
	
	def color_not_found_message(self, color):
		object_message = 'Object ' + color + ' wasnt found in the image.'
		self.object_publisher.publish(object_message)
				
				
	def find_contours(self, color_key, color_mask= None, contours = None):
		if not contours:
			contours, _ = cv.findContours(color_mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
		
		if len(contours):
			c = max(contours, key=cv.contourArea)
			((x,y), radius) = cv.minEnclosingCircle(c)
			
			color_message = 'radius of the ' + color_key + ' contour circle is: ' + str(radius)
			self.color_publisher.publish(color_message)
			
			if radius > 5:
				center = (int(x), int(y))
				# draw a circle on the contour you're identifying
				cv.circle(self.image, center, int(radius), (255, 255, 255), 1)
				self.color_search[color_key] = True
				return radius
		else:
			self.color_search[color_key] = False
			self.color_not_found_message(color_key)
			
		
	
	def find_colors(self, red_mask):
		# find contours for red
		self.find_contours('Red', red_mask)
			
				
	def only_green(self):
				
		return True if (self.color_search['Green'] and not self.color_search['Red'] and not self.color_search['Blue']) else False
		
		
	def find_centroid(self, contours):
		# Use the max() method to find the largest contour
		c = max(contours, key=cv.contourArea)
		
		M = cv.moments(c)
		# x and y coordinates of the centroid of the image blob
		cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
		
		return cx, cy

	
	def image_callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			
			# Convert the rgb image into a hsv image
			cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
			
			red_mask, blue_mask, green_mask = self.extract_colors(cv_image)
	
			# Find the contours that appear within the certain colours mask using the cv2.findContours() method
			# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
			#ret, thresh = cv.threshold(green_mask, 40, 255, 0)
			self.find_colors(red_mask, blue_mask)
			contours, _ = cv.findContours(green_mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
			
			if len(contours):
				# Loop over the contours
				# There are a few different methods for identifying which contour is the biggest
				# Loop throguht the list and keep track of which contour is biggest or
				
				cx, cy = self.find_centroid(contours)	# returns centroid coordinates
				
				height, width, depth = cv_image.shape
				
				#Check if the area of the shape you want is big enough to be considered
				# If it is then change the flag for that colour to be True(1)
				radius = self.find_contours('Green', contours=contours)
	
				#Check if a flag has been set for the stop message
				diff = -float((cx-(width/2))/100)
				# turn towards the goal
				angular = (0,0,diff) 
				
				if radius >= 10 and radius < 50:
					# Too far away from object, need to move forwards
					linear = (0.2,0,0)
					self.move(linear, angular)
					
			else:
				self.color_search['Green'] = False
				self.color_not_found_message('Green')
			
			# Be sure to do this for the other colour as well
		except CvBridgeError as e:
			print(e)
		cv.namedWindow('Camera Feed')
		cv.imshow('Camera Feed', self.image)
		cv.waitKey(3)
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	try:
		# Instantiate your class
		# And rospy.init the entire node
		rospy.init_node('Objectdentifier')
		
		cI = colourIdentifier()
		# Ensure that the node continues running with rospy.spin()
		# You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
		# Remember to destroy all image windows before closing node
		cv.destroyAllWindows()
	
		rospy.spin()
	
	except rospy.ROSInterruptException:
		pass

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)


