

import rospy
import cv2 as cv 
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid

import yaml

map_path = './src/group27/project/example/map/'


class OccupancyGridMap:
	def __init__(self):
		
		ogm_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.load_map, queue_size=1)
		
		with open(map_path + 'project_map.yaml', 'r') as stream:
			try:
				map_info = yaml.safe_load(stream)
				
				self.resolution = map_info['resolution']
				self.origin = map_info['origin']
				self.occupied_thresh = map_info['occupied_thresh']
				self.free_thresh = map_info['free_thresh']
				
				
			except yaml.YAMLError as e:
				print(e)
	
	
	def load_map(self, ogm):
		
		project_map = cv.imread(map_path + 'project_map.pgm')
		(height, width, depth) = project_map.shape
		
		og = np.asarray(ogm.data)
		og = np.reshape(og, (width, height))
		
		self.data = og
		
	
	def get_occupancy_value(self, point):
		    
		x, y = point
		x_index, y_index = self.get_index_from_location(x, y)
		
		return self.data[x_index][y_index]
		
	
	def get_index_from_location(self, x, y):
		x_index = int(round(x/self.resolution))
		y_index = int(round(y/self.resolution))


		return x_index, y_index
		
		
	def get_location_from_index(self, x_index, y_index):
		x = x_index*self.resolution
		y = y_index*self.resolution
		
		return x, y





































	def plot(self, alpha=1, min_val=0, origin='lower'):
		plt.imshow(self.data, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha, cmap='gray')
		plt.draw()
		plt.show()

