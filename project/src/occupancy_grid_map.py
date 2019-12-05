import cv2 as cv 
import numpy as np

from nav_msgs.msg import OccupancyGrid



map_path = './src/group27/project/example/map/project_map.pgm'
#~ ogm_subscriber = rospy.Subscriber('/map', OccupancyGrid, load_map, queue_size=1)


def load_map(ogm):
	
	project_map = cv.imread(map_path)
	(height, width, depth) = project_map.shape
	
	og = np.asarray(ogm.data)
	og = np.reshape(og, (width, height))
	
	og = 255*og
	cv.imshow('OGM', og)
	cv.waitKey(0)
	
	print(og.shape)
















