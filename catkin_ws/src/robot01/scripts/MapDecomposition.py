#!/usr/bin/env python

"""
	# http://wiki.ros.org/map_server
	# http://wiki.ros.org/rospy/Overview/Services

"""
import math
import rospy
import roslib
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

class MapDecomposition:
	"""docstring for ClassName"""
	def __init__(self,robot_dimensions_xyz = []):
		rospy.init_node('Grid_Publisher')
		rospy.loginfo("Starting Map Grid Publisher")
		self.mapPublisher = rospy.Publisher('/MapDecomposition', OccupancyGrid, queue_size=1)
		self.updateRate = rospy.Rate(0.2)
		self.grid = OccupancyGrid()
		self.loadedMap = self.get_map().map
		if robot_dimensions_xyz == []:
			self.robot_dimensions_xyz = rospy.get_param('/robot/dimensions_xyz', [1.0,1.0,0.25])
		else:
			self.robot_dimensions_xyz = robot_dimensions_xyz
		self.grid_cell_size = self.__calc_cell_size()
		self.grid.info.width = int(math.ceil(self.loadedMap.info.width / float(self.grid_cell_size)))
		self.grid.info.height = int(math.ceil(self.loadedMap.info.height / float(self.grid_cell_size)))
		self.grid.info.resolution = self.grid_cell_size / 10.0
		self.grid.info.origin.position.x = self.loadedMap.info.origin.position.x
		self.grid.info.origin.position.y = self.loadedMap.info.origin.position.y
		self.grid.info.origin.orientation.w = 1
		self.get_decomposition()
		while not rospy.is_shutdown():
			self.mapPublisher.publish(self.grid)
			self.updateRate.sleep()
		rospy.spin()

	def __calc_cell_size(self):
		hypoteamus = math.hypot((self.robot_dimensions_xyz[0]/2.0),(self.robot_dimensions_xyz[1]/2.0))
		grid_cell_size = math.ceil(hypoteamus / self.loadedMap.info.resolution)
		grid_cell_size = int(grid_cell_size)
		return grid_cell_size

	def get_map(self):
		rospy.wait_for_service('static_map')
		try:
			map_storage = rospy.ServiceProxy('static_map', GetMap)
			return map_storage()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def __check_cell(self,i,j):
		row_jumper = 0
		for k in range(self.grid_cell_size):
			for l in range(self.grid_cell_size):
				try:
					if (self.loadedMap.data[i+j+l+row_jumper] !=0 or j == self.loadedMap.info.width-1):
						self.grid.data.append(100)
						return
					elif(l == self.grid_cell_size-1):
						row_jumper = row_jumper + self.loadedMap.info.width
				except IndexError:
					self.grid.data.append(100)
					return
		self.grid.data.append(0)


	def get_decomposition(self):
		step = self.loadedMap.info.width*self.grid_cell_size
		for i in range(0,len(self.loadedMap.data), step):
			for j in range(0,self.loadedMap.info.width,self.grid_cell_size):
				self.__check_cell(i,j)

map_grid = MapDecomposition()