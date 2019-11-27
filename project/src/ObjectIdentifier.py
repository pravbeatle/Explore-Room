#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2 as cv
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rospy_message_converter import message_converter
import json


class colourIdentifier():

	def __init__(self):
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.color_sensitivity = 10
		
		self.image = None	# the image that we continuisly process on top of
		self.cv_image = None
		
		self.bot_status = ''

		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.cv_bridge = CvBridge()
		
		self.camera_subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
		# We covered which topic to subscribe to should you wish to receive image data
		
		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
		self.rate = rospy.Rate(10) 		#10hz
		
		# publish and subcribe to relevant topics
		self.object_color_publisher = rospy.Publisher('object_detection/color/json', String, queue_size=10)
		self.object_publisher = rospy.Publisher('object_detection/object/json', String, queue_size=10)
		
		self.status_subscriber = rospy.Subscriber('explorer_bot/status', String, self.status_callback, queue_size=1)
				
	
	def extract_colors(self, cv_image):
		# Set the upper and lower bounds for the two colours you wish to identify
		hsv_green_lower = np.array([40, 100, 0])
		hsv_green_upper = np.array([80, 255, 100])
		# Red has lower and upper bounds split on eithr side
		hsv_red_lower = np.array([0, 100, 100])
		hsv_red_upper = np.array([self.color_sensitivity, 255, 120])
		
		hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
		# Filter out everything but particular colours using the cv2.inRange() method
		green_mask = cv.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
		red_mask = cv.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
		# To combine the masks you should use the cv2.bitwise_or() method
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours
		combined_masks = cv.bitwise_or(red_mask, green_mask)
		# Apply the mask to the original image using the cv2.bitwise_and() method
		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image. 
		self.image = cv.bitwise_and(cv_image, cv_image, mask=combined_masks)
		
		return red_mask, green_mask
		
				
		
	def find_circles(self):
		
		image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
		
		image = cv.medianBlur(image,5)
		
		circles = cv.HoughCircles(image, cv.cv.CV_HOUGH_GRADIENT, 1, 20,
		                           param1=20, param2=10, minRadius=5, maxRadius=50)
		                           
		
		if circles is not None:
			circles = np.uint8(np.around(circles))
			
			for i in circles[0,:]:
			    # draw the outer circle
			    cv.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
			    # draw the center of the circle
			    cv.circle(image,(i[0],i[1]),2,(0,0,255),3)
			
			return True 
		else:
			return False 
					
				
	def find_contours(self, color_key, color_mask):
		contours, _ = cv.findContours(color_mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
				
		if len(contours):
			c = max(contours, key=cv.contourArea)
			((x,y), radius) = cv.minEnclosingCircle(c)
			cx, cy = self.find_centroid(c)
			diff = self.find_diff(cx)
			
			if radius > 5:
				#~ center = (int(x), int(y))
				#~ # draw a circle on the contour you're identifying
				#~ cv.circle(self.image, center, int(radius), (255, 255, 255), 1)
				
				object_message = {
								'object_color': color_key,
								'radius':       radius,
								'angle_from_centroid': diff,
								'circle_found': self.find_circles()
							}
							 
				object_message = json.dumps(object_message)
				self.object_color_publisher.publish(object_message)
		
	
	def find_colors(self, red_mask, green_mask):
		# find contours for red
		#~ self.find_contours('Red', red_mask)
		self.find_contours('Green', green_mask)
		
		
	def find_centroid(self, c):
		M = cv.moments(c)
		if M['m00'] == 0:
			cx, cy = 0, 0
		else:
			# x and y coordinates of the centroid of the image blob
			cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
		
		return cx, cy
		
		
	def find_diff(self, cx):
		height, width, channels = self.cv_image.shape
		
		return float((cx - (width/2))/100)
		
	
	def status_callback(self, data):
		data = message_converter.convert_ros_message_to_dictionary(data)['data']
		data = json.loads(data)
		
		if data['status'] == 'focus_done':
			cv.imwrite(data['file_path'], self.cv_image)
		
		self.bot_status = data['status']
			
	
	def faces_found(self, gray):
		
		face_cascade = cv.CascadeClassifier('./src/group27/project/src/haarcascade_frontalface_default.xml')
		faces = face_cascade.detectMultiScale(gray, 1.2, 5)
				
		for (x,y,w,h) in faces:
		
			cv.rectangle(self.cv_image,(x,y),(x+w,y+h),(255,0,0),2)
				
		return (True, x, y) if (len(faces) > 0) else (False, None, None)
	
	
	def find_posters(self):
		
		gray = cv.cvtColor(self.cv_image, cv.COLOR_RGB2GRAY)
		blurred_gray = cv.blur(gray, (7,7))
		
		edges = cv.Canny(blurred_gray, 100, 200)
		
		_, thresh = cv.threshold(edges, 200, 255, cv.THRESH_BINARY)
		self.image = thresh
		
		contours, _ = cv.findContours(thresh, 1, 2)
		
		if len(contours):
			
			c = max(contours, key=cv.contourArea)
			((x,y), radius) = cv.minEnclosingCircle(c)
			
			#~ cv.drawContours(self.image, [c], -1, (255, 0, 0), 2)
			center = (int(x), int(y))
				#~ # draw a circle on the contour you're identifying
			cv.circle(self.image, center, int(radius), (255, 0, 0), 2)
				
		
			approx = cv.approxPolyDP(c, 0.01*cv.arcLength(c, True), True)
			face_and_pos = self.faces_found(gray)
			
			if len(approx) > 4 and face_and_pos[0]:
				print('FOUND A POSTER!!', len(approx))
				diff = self.find_diff(face_and_pos[1])
				
				object_message = {
								'radius':       radius,
								'angle_from_centroid': diff,
								'faces_found': True
							}
				object_message = json.dumps(object_message)
				self.object_publisher.publish(object_message)
	
	
	def image_callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			
			# Convert the rgb image into a hsv image
			cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
			self.cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
			
			if self.bot_status == '':
				red_mask, green_mask = self.extract_colors(cv_image)
		
				# Find the contours that appear within the certain colours mask using the cv2.findContours() method
				# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
				#ret, thresh = cv.threshold(green_mask, 40, 255, 0)
				self.find_colors(red_mask, green_mask)
				
			elif self.bot_status == 'room_scan':
				
				self.find_posters()
				
									
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


