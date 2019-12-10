
import rospy
import cv2 as cv 
import numpy as np
import matplotlib.pyplot as plt
from exceptions import IndexError

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

import yaml
import math
from itertools import permutations

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
		
		
		with open(map_path + 'project_map.yaml', 'r') as stream:
			try:
				map_info = yaml.safe_load(stream)
				
				self.free_thresh = map_info['free_thresh']
				self.occupied_thresh = map_info['occupied_thresh']
				
			except yaml.YAMLError as e:
				print(e)
		
		
		self.origin = Point()
		
		self.origin.x = ogm.info.origin.position.x
		self.origin.y = ogm.info.origin.position.y
		
		self.data = ogm.data
	
	
	def are_indices_within_map(self, i, j):
		
		return 0 <= i < self.height and 0 <= j < self.width	
	
	
	def get_by_index(self, i, j):
		
		if not self.are_indices_within_map(i, j):
			raise IndexError()
		
		return self.data[i*self.width + j]
		
	
	def get_occupancy_value_by_location(self, x, y):
		
		i, j = self.get_index_from_location(x, y)
				
		return self.get_by_index(i, j)
	
	
	def get_occupancy_value_by_index(self, x, y):
				
		return self.get_by_index(x, y)
		
	
	def get_index_from_location(self, x, y):
		
		i = int((y - self.origin.y) / self.resolution)
		j = int((x - self.origin.x) / self.resolution)

		return i, j
		
		
	def get_location_from_index(self, x_index, y_index):
		
		y = x_index*self.resolution + self.origin.y
		x = y_index*self.resolution + self.origin.x
		
		return x, y


	def find_wall_by_direction(self, x, y, dir_x, dir_y):
		
		occupancy_value = self.get_occupancy_value_by_index(x, y)
		
		while occupancy_value <= self.free_thresh:
			x += dir_x
			y += dir_y
			
			occupancy_value = self.get_occupancy_value_by_index(x, y)
		
		# keep going back by 15 cells from wall until you find a free spot
		while True:
			x += (-1*dir_x*15)
			y += (-1*dir_y*15)
			
			occupancy_value = self.get_occupancy_value_by_index(x, y)
			
			if occupancy_value <= self.free_thresh:
				break
		
		return x, y 
		
	
	def dist(self, x1, y1, x2, y2):
		
		return math.sqrt( (x1-x2)**2 + (y1-y2)**2 )
			
		 
	def sample_wall_points(self, center_x, center_y):
		
		center_x_index, center_y_index = self.get_index_from_location(center_x, center_y)
		
		sampled_points = []
		directions = list(permutations([-1, 0, 1], 2))
		directions.append((1, 1))
		directions.append((-1,-1))
		
		for direction in directions:
			x, y = self.find_wall_by_direction(center_x_index, center_y_index, direction[0], direction[1])
			
			x_loc, y_loc = self.get_location_from_index(x, y)
			
			sampled_points.append(
				{
					'point': (x_loc, y_loc), 
					'dist_from_center': self.dist(center_x, center_y, x_loc, y_loc), 
					'visited': False
				}
			)
		
		sampled_points = sorted(sampled_points, key = lambda x: x['dist_from_center'])
		del sampled_points[-1]
		
		return sampled_points


	def plot(self, alpha=1, min_val=0, origin='lower'):
		plt.imshow(self.data, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha, cmap='gray')
		plt.draw()
		plt.show()

