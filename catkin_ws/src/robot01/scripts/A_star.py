#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# 

"""
import math
import rospy
from Queue import PriorityQueue

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
	def __init__(self, grid, start_point_xy, goal_points_xy = [[0,0],[0,0]]):
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
		goal_nodes = []
		start_node = Node(self.start_point_xy)
		closest_to_start = PriorityQueue()
		for i in range(len(goals)): 
			goal_nodes.append(Node(goals[i]))
		for i in range(len(goal_nodes)):
			for j in range(len(goal_nodes)-i):
				goal_nodes[i] = self.h(goal_nodes[i],start_node)
		

	def h(self,node,goal):
		dx = abs(node.x - goal.x)
		dy = abs(node.y - goal.y)
		return dx+dy


w, h = 3, 2
Matrix = [[0 for x in range(w)] for y in range(h)]
