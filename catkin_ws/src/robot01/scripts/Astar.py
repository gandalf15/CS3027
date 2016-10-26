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
		#while not self.prioritizedGoalNodes.empty():
		#	path = self.find_path(self.grid, self.startNode, self.prioritizedGoalNodes.get())
		#	self.path.append(path)

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

	def find_path(self, grid=OccupancyGrid(), start_point_xy=[], end_point_xy=[]):		
		unvisitedQueue = PriorityQueue()
		visitedNodes = []
		startNode = Node(position_xy=start_point_xy, priority = 0)
		startNode.g = 0
		goalNode = Node(position_xy=end_point_xy)
		unvisitedQueue.put(startNode)
		path = []

		while not unvisitedQueue.empty():
			currentNode = unvisitedQueue.get()
			visitedNodes.append(currentNode)
			#print "cur node pose: ", currentNode.position_xy
			#print "goal node pose: ", goalNode.position_xy
			if currentNode.position_xy == goalNode.position_xy:
				print "cur position node: ", currentNode.position_xy
				print "sratr node pose: ", startNode.position_xy
				while startNode.position_xy != currentNode.position_xy or currentNode.parent == None:
					path.insert(0, currentNode.position_xy)
					currentNode = currentNode.parent
				return path
				# track back parents and get the path
				break
			for childNode in self.__get_children(currentNode, visitedNodes):
				newCostG = currentNode.g + 1	# f(g) = 1
				if childNode not in visitedNodes or newCostG < childNode.g:
					childNode.g = newCostG
					childNode.priority = newCostG + self.h(childNode, goalNode)
					childNode.parent = currentNode
					unvisitedQueue.put(childNode)
		print "path does not exist!"

	
	def __get_children(self, currentNode, visitedNodes):
		pose = currentNode.position_xy
		childNodes = []
		for i in range(len(visitedNodes)):
			if visitedNodes[i].position_xy == [pose[0]-1,pose[1]]:
				childNodes.append(visitedNodes[i])
				break
			elif i == len(visitedNodes)-1:
				if self.__get_occupancy_value([pose[0]-1,pose[1]]) == 0:
					childNodes.append(Node([pose[0]-1,pose[1]]))
		for i in range(len(visitedNodes)):
			if visitedNodes[i].position_xy == [pose[0]+1,pose[1]]:
				childNodes.append(visitedNodes[i])
				break
			elif i == len(visitedNodes)-1:
				if self.__get_occupancy_value([pose[0]+1,pose[1]]) == 0:
					childNodes.append(Node([pose[0]+1,pose[1]]))
		for i in range(len(visitedNodes)):
			if visitedNodes[i].position_xy == [pose[0],pose[1]-1]:
				childNodes.append(visitedNodes[i])
				break
			elif i == len(visitedNodes)-1:
				if self.__get_occupancy_value([pose[0],pose[1]-1]) == 0:
					childNodes.append(Node([pose[0],pose[1]-1]))
		for i in range(len(visitedNodes)):
			if visitedNodes[i].position_xy == [pose[0],pose[1]+1]:
				childNodes.append(visitedNodes[i])
				break
			elif i == len(visitedNodes)-1:
				if self.__get_occupancy_value([pose[0],pose[1]+1]) == 0:
					childNodes.append(Node([pose[0],pose[1]+1]))
		return childNodes

	def __get_occupancy_value(self,coordinates = []):
		if coordinates:
			x = int(coordinates[0]*self.grid.info.resolution - self.grid.info.origin.position.x)
			y = int(coordinates[1]*self.grid.info.resolution - self.grid.info.origin.position.y)
			index = x+y*self.grid.info.width
			try:
				return self.grid.data[index]
			except IndexError:
				return 1


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