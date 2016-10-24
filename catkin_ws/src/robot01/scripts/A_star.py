#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# 

"""
import math
import rospy
from sys import maxint
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid

class Node:
	"""docstring for Node"""
	def __init__(self, position_xy = [], parent = None, children = [], priority = None):
		self.position_xy = position_xy
		self.h = None
		self.g = None
		self.f = None
		self.parent = parent
		self.children = children
		self.priority = priority

	def __cmp__(self, other):
		return cmp(self.priority, other.priority)
		
class Astar:
	"""docstring for Astar"""
	def __init__(self,start_point_xy, goal_points_xy, grid = OccupancyGrid()):
		self.grid = grid
		self.start_point_xy = start_point_xy
		if goal_points_xy:
			pass
		else:
			print "No goals set!"
			return 1
		self.visitedQueue = []
		self.priorityQueue = PriorityQueue()
		self.solvedPath = []
		self.prioritizedGoals= []
		goalNodes = PriorityQueue()
		for i in range(len(goal_points_xy)): 
			goalNodes.put(Node(goal_points_xy[i], priority=0))
		self.__prioritize_nodes_heur(Node(self.start_point_xy, priority=0), goalNodes)
		
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

try:
	astar = Astar(start_point_xy = [-10,-10], goal_points_xy=[[-10,-11],[-2,2],[3,8],[20,80],[1,1]])
	print "job done"
	print astar.prioritizedGoals
except KeyboardInterrupt:
	pass

w, h = 3, 2
Matrix = [[0 for x in range(w)] for y in range(h)]
