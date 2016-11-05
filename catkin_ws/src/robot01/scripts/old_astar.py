#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# http://www.redblobgames.com/pathfinding/a-star/introduction.html

"""
import math
from Queue import PriorityQueue

def heur(self,pose,goal):
	dx = abs(pose[0] - goal[0])
	dy = abs(pose[1] - goal[1])
	return dx+dy

def astar(graph_of_map, start_pose, goal_pose):		
		unvisitedQueue = PriorityQueue()	#open list
		startNode = graph_of_map.get_node(start_pose)
		goalNode = graph_of_map.get_node(goal_pose)
		unvisitedQueue.put((0,startNode))
		parents = {}
		cost = {}
		parents[startNode] = None
		cost[startNode] = 0
		path = []
		while not unvisitedQueue.empty():
			currentNode = unvisitedQueue.get()[1]
			if currentNode == goalNode:
				while currentNode != startNode:
					path.insert(0, currentNode.get_position())
					currentNode = parents[currentNode]
				#print path
				return path

			for neighbour in currentNode.get_neighbours(currentNode):
				newCost = cost[currentNode] + 1	# f(g) = 1
				if neighbour not in cost or newCost < cost[neighbour]:
					cost[neighbour] = newCost
					priority = newCost + heur(neighbour.get_position(), goalNode.get_position())
					parents[neighbour] = currentNode
					unvisitedQueue.put((priority,neighbour))
		print "Could not find path!!!"

def get_children(pose, parentPose):
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