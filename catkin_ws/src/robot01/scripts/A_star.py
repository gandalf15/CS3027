#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# 

"""
import math
import rospy
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
	def __init__(self, grid = OccupancyGrid(), start_point_xy, goal_points_xy = [[0,0],[0,0]]):
		self.grid = grid
		self.start_point_xy = start_point_xy
		if not self.goal_points_xy != [[0,0][0,0]]:
			self.prioritized_goal_nodes = prioritize_goals(goal_points_xy)
		else:
			print "No goals set!"
			return 1
		self.visitedQueue = []
		self.priorityQueue = PriorityQueue()
		self.solvedPath = []
		self.priorityQueueGoals = PriorityQueue()
		
	def prioritize_goals(goals):
		startNode = Node(self.start_point_xy)
		goalNodes = []
		unvisitedNodes = PriorityQueue()
		for i in range(len(goals)): 
			goalNodes.append(Node(goals[i]))
		for i in range(len(goalNodes)):
			goalNodes[i].priority = self.h(nodes[i],startNode)
			unvisitedNodes.put(goalNodes[i])
		self.priorityQueueGoals.put(unvisitedNodes.get())
		for i in range(len(unvisitedNodes)):
			node = 

	def h(self,node,goal):
		dx = abs(node.x - goal.x)
		dy = abs(node.y - goal.y)
		return dx+dy


w, h = 3, 2
Matrix = [[0 for x in range(w)] for y in range(h)]
