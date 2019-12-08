

import rospy
import cv2 as cv 
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

import yaml

map_path = './src/group27/project/example/map/'


class OccupancyGridMap:
	def __init__(self):
		
		ogm_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.load_map, queue_size=1)
		
	
	
	def load_map(self, ogm):
		
		print(ogm.info)
		self.ogm = ogm
		self.width = ogm.info.width
		self.height = ogm.info.height
		self.resolution = ogm.info.resolution
		
		self.origin = Point()
		
		self.origin.x = ogm.info.origin.position.x
		self.origin.y = ogm.info.origin.position.y
		
		self.data = ogm.data
		
	
	def get_occupancy_value(self, x, y):
		    
		i, j = self.get_index_from_location(x, y)
		
		print('map point of indices : ', self.get_location_from_index(i, j))
		
		return self.data[i*self.width + j]
		
	
	def get_index_from_location(self, x, y):
		i = int((y - self.origin.y) / self.resolution)
		j = int((x - self.origin.x) / self.resolution)


		return i, j
		
		
	def get_location_from_index(self, x_index, y_index):
		y = x_index*self.resolution + self.origin.y
		x = y_index*self.resolution + self.origin.x
		
		return x, y





































	def plot(self, alpha=1, min_val=0, origin='lower'):
		plt.imshow(self.data, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha, cmap='gray')
		plt.draw()
		plt.show()

