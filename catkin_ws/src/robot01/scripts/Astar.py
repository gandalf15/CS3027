#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# http://www.redblobgames.com/pathfinding/a-star/introduction.html

"""
import math
import rospy
from sys import maxint
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid

class Node:
	"""docstring for Node"""
	def __init__(self, position_xy,cost = 0, parent = None):
		self.position_xy = position_xy
		self.cost = cost
		self.parent = parent

		
class Astar:
	"""docstring for Astar"""
	def __init__(self,start_point_xy, goal_points_xy, grid):
		self.grid = grid
		self.startNode = Node(start_point_xy)
		nodeQueue = PriorityQueue()
		for i in range(len(goal_points_xy)): 
			nodeQueue.put((0,Node(goal_points_xy[i])))
		self.prioritizedGoalNodes = PriorityQueue()
		self.__prioritize_goals(self.startNode, nodeQueue)
		self.path = []
		self.unvisitedQueue = PriorityQueue()
		startNode = self.startNode
		while not self.prioritizedGoalNodes.empty():
			goalNode = self.prioritizedGoalNodes.get()[1]
			self.find_path((startNode.position_xy[0],startNode.position_xy[1]), (goalNode.position_xy[0],goalNode.position_xy[1]))
			startNode = goalNode
		print self.path
		self.robot_dimensions_xyz = rospy.get_param('/robot/dimensions_xyz', [1.0,1.0,0.25])
		self.grid_cell_size = self.__calc_cell_size()

	def __prioritize_goals(self, start_node, goal_queue):
		remainingNodes = PriorityQueue()
		if goal_queue.qsize() > 1:
			for i in range(goal_queue.qsize()):
				node = goal_queue.get()[1]
				priority = self.h(node,start_node)
				remainingNodes.put((priority,node))
			closestNode = remainingNodes.get()
			self.prioritizedGoalNodes.put(closestNode)
			self.__prioritize_goals(closestNode[1],remainingNodes)
		else:
			lastNode = goal_queue.get()[1]
			self.prioritizedGoalNodes.put((maxint,lastNode))

	def h(self,node,goal):
		dx = abs(node.position_xy[0] - goal.position_xy[0])
		dy = abs(node.position_xy[1] - goal.position_xy[1])
		return dx+dy

	def heur(self,child,goal):
		dx = abs(child[0] - goal[0])
		dy = abs(child[1] - goal[1])
		return dx+dy

	def find_path(self, start, goal):		
		unvisitedQueue = PriorityQueue()	#open list
		unvisitedQueue.put((0,start))
		parents = {}
		cost = {}
		parents[start] = None
		cost[start] = 0
		path = []
		while not unvisitedQueue.empty():
			currentPose = unvisitedQueue.get()[1]
			if currentPose == goal:
				while currentPose != start:
					path.insert(0, [currentPose[0],currentPose[1]])
					currentPose = parents[currentPose]
				self.path.append(path)
				return

			for child in self.__get_children(currentPose, parents[currentPose]):
				newCost = cost[currentPose] + 1	# f(g) = 1
				if child not in cost or newCost < cost[child]:
					cost[child] = newCost
					priority = newCost + self.heur(child, goal)
					parents[child] = currentPose
					unvisitedQueue.put((priority,child))
		print "Could not find path!!!"

	
	def __get_children(self, pose, parentPose):
		children = []
		accuracy = 1
		if self.__get_occupancy_value([pose[0]-accuracy,pose[1]]) == 0 and (pose[0]-accuracy,pose[1]) != parentPose:
			children.append((pose[0]-accuracy,pose[1]))
		if self.__get_occupancy_value([pose[0]+accuracy,pose[1]]) == 0 and (pose[0]+accuracy,pose[1]) != parentPose:
			children.append((pose[0]+accuracy,pose[1]))
		if self.__get_occupancy_value([pose[0],pose[1]-accuracy]) == 0 and (pose[0],pose[1]-accuracy) != parentPose:
			children.append((pose[0],pose[1]-accuracy))
		if self.__get_occupancy_value([pose[0],pose[1]+accuracy]) == 0 and (pose[0],pose[1]+accuracy) != parentPose:
			children.append((pose[0],pose[1]+accuracy))
		return children

	def __get_occupancy_value(self,coordinates):
		if coordinates:
			x = int(round((coordinates[0] - self.grid.info.origin.position.x)/self.grid.info.resolution))
			y = int(round((coordinates[1] - self.grid.info.origin.position.y)/self.grid.info.resolution))
			index = x+y*self.grid.info.width
			try:
				return self.grid.data[index]
			except IndexError:
				return 1


	def __calc_cell_size(self):
		hypoteamus = math.hypot((self.robot_dimensions_xyz[0]/2.0),(self.robot_dimensions_xyz[1]/2.0))
		grid_cell_size = math.ceil(hypoteamus / self.grid.info.resolution)
		grid_cell_size = int(grid_cell_size)
		return grid_cell_size

	def __check_cell(self,i,j):
		row_jumper = 0
		for k in range(self.grid_cell_size):
			for l in range(self.grid_cell_size):
				try:
					if (self.grid.data[i+j+l+row_jumper] !=0 or j == self.grid.info.width-1):
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


"""
try:
	astar = Astar(start_point_xy = [-10,-10], goal_points_xy=[[-10,-11],[-2,2],[3,8],[20,80],[1,1]], grid=OccupancyGrid())
	print "job done"
	while not astar.prioritizedGoalNodes.empty():
		print astar.prioritizedGoalNodes.get()[1].position_xy
except KeyboardInterrupt:
	pass

w, h = 3, 2
Matrix = [[0 for x in range(w)] for y in range(h)]
"""
