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
	def __init__(self, position_xy = [], parent = None, priority = None):
		self.position_xy = position_xy
		self.g = maxint
		self.parent = parent
		self.priority = priority

	def __cmp__(self, other):
		return cmp(self.priority, other.priority)
		
class Astar:
	"""docstring for Astar"""
	def __init__(self,start_point_xy = [], goal_points_xy = [], grid = OccupancyGrid()):
		self.grid = grid
		self.start_point_xy = start_point_xy
		if goal_points_xy:
			pass
		else:
			print "No goals set!"
			return 1
		self.solvedPath = []
		self.prioritizedGoals= []
		goalNodes = PriorityQueue()
		for i in range(len(goal_points_xy)): 
			goalNodes.put(Node(goal_points_xy[i]))
		self.__prioritize_nodes_heur(Node(self.start_point_xy), goalNodes)
		
	def __prioritize_nodes_heur(self, start_node, goal_nodes = PriorityQueue()):
		leftNodes = PriorityQueue()
		if goal_nodes.qsize() > 1:
			for i in range(goal_nodes.qsize()):
				node = goal_nodes.get() 
				node.priority = self.h(node,start_node)
				leftNodes.put(node)
			closestNode = leftNodes.get()
			self.prioritizedGoals.append(closestNode.position_xy)
			self.__prioritize_nodes_heur(closestNode,leftNodes)
		else:
			lastNode = goal_nodes.get()
			lastNode.priority = maxint
			self.prioritizedGoals.append(lastNode.position_xy)

	def h(self,node,goal):
		dx = abs(node.position_xy[0] - goal.position_xy[0])
		dy = abs(node.position_xy[1] - goal.position_xy[1])
		return dx+dy

	def find_path(grid=OccupancyGrid(), start_point_xy=[], end_point_xy=[]):		
		unvisitedQueue = PriorityQueue()
		visitedNodes = set()
		startNode = Node(position_xy=start_point_xy, priority = 0)
		startNode.g = 0
		goalNode = Node(position_xy=end_point_xy)
		unvisitedNodes.put(startNode)

		while not unvisitedNodes.empty():
			currentNode = unvisitedNodes.get()
			visitedNodes.add(currentNode)
			if currentNode.position_xy == goalNode.position_xy:
				# track back parents and get the path
				break

			for childNode in self.__get_children(currentNode, visitedNodes):
				newCostG = currentNode.g + 1	# f(g) = 1
				if childNode not in visitedNodes or newCostG < childNode.g:
					childNode.g = newCost
					childNode.priority = newCostG + self.h(childNode, goalNode)
					childNode.parent = currentNode
					unvisitedNodes.put(childNode)

	
	def __get_children(self, currentNode, visitedNodes):
		pose = currentNode.position_xy
		childrenNode = []
		if self.__get_occupancy_value([pose[0]-1,pose[1]]) == 0:
			leftChild = Node([pose[0]-1,pose[1]])
			if leftChild not in visitedNodes:
				childrenNode.append(leftChild)
			else:
				childrenNode.append(visitedNodes.remove(leftChild))
		if self.__get_occupancy_value([pose[0]+1,pose[1]]) == 0:
			rightChild = Node([pose[0]+1,pose[1]])
			childrenNode.append(rightChild)
		if self.__get_occupancy_value([pose[0],pose[1]-1]) == 0:
			upChild = Node([pose[0],pose[1]-1])
			childrenNode.append(upchild)
		if self.__get_occupancy_value([pose[0],pose[1]-1]) == 0:
			downChild = Node([pose[0],pose[1]-1])
			childrenNode.append(downChild)
		return childrenNode

	def __get_occupancy_value(self,coordinates = []):
		if coordinates:
			x = coordinates[0] - self.grid.info.origin.position.x
			y = coordinates[1] - self.grid.info.origin.position.y
			index = x+y*self.grid.info.width
			try:
				return self.grid.data[index]
			except IndexError:
				return 1



try:
	astar = Astar(start_point_xy = [-10,-10], goal_points_xy=[[-10,-11],[-2,2],[3,8],[20,80],[1,1]])
	print "job done"
	print astar.prioritizedGoals
except KeyboardInterrupt:
	pass

w, h = 3, 2
Matrix = [[0 for x in range(w)] for y in range(h)]
