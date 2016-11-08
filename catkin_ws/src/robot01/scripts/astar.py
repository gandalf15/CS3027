#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# http://www.redblobgames.com/pathfinding/a-star/introduction.html

"""
import math
from sys import maxint
from Queue import PriorityQueue

def prioritize_goals(start_pose, goal_queue, sorted_goals = []):
	remainingPose = PriorityQueue()
	if goal_queue.qsize() > 1:
		for i in range(goal_queue.qsize()):
			pose = goal_queue.get()[1]
			priority = heuristic_cost(pose,start_pose)
			remainingPose.put((priority,pose))
		closestPose = remainingPose.get()
		sorted_goals.append(closestPose[1])
		return prioritize_goals(closestPose[1],remainingPose, sorted_goals)
	else:
		lastPose = goal_queue.get()[1]
		sorted_goals.append(lastPose)
		return sorted_goals

def heuristic_cost(pose,goal):
	return abs(pose[0] - goal[0]) + abs(pose[1] - goal[1])

def find_path(start, goal, grid):		
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
			return path

		for child in get_children(currentPose, parents[currentPose], grid):
			newCost = cost[currentPose] + 1	# f(g) = 1
			if child not in cost or newCost < cost[child]:
				cost[child] = newCost
				priority = newCost + heuristic_cost(child, goal)
				parents[child] = currentPose
				unvisitedQueue.put((priority,child))
	print "Could not find path!!!"
	return False


def get_children(pose, parentPose, grid):
	children = []
	accuracy = 1

	if get_occupancy_value([pose[0]-accuracy,pose[1]], grid) and (pose[0]-accuracy,pose[1]) != parentPose:
		children.append((pose[0]-accuracy,pose[1]))
	if get_occupancy_value([pose[0]+accuracy,pose[1]], grid) and (pose[0]+accuracy,pose[1]) != parentPose:
		children.append((pose[0]+accuracy,pose[1]))
	if get_occupancy_value([pose[0],pose[1]-accuracy], grid) and (pose[0],pose[1]-accuracy) != parentPose:
		children.append((pose[0],pose[1]-accuracy))
	if get_occupancy_value([pose[0],pose[1]+accuracy], grid) and (pose[0],pose[1]+accuracy) != parentPose:
		children.append((pose[0],pose[1]+accuracy))
	"""
	for y in range(3):
		for x in range(3):
			currentX = pose[0]-accuracy+x
			currentY = pose[1]-accuracy+y
			if (currentX,currentY) != parentPose:
				realX = int(round((currentX - grid.info.origin.position.x)/grid.info.resolution))
				realY = int(round((currentY - grid.info.origin.position.y)/grid.info.resolution))
				index = realX+realY*grid.info.width
				if cell_clear(index, grid):
					children.append((currentX,currentY))
	"""
	return children

def cell_clear(index, grid):
	row_jumper = 0
	beginIndex = (index-8)-grid.info.width*8
	lineNo = math.ceil(beginIndex/grid.info.width-1.0)
	for i in range(16):
		for j in range(16):
			try:
				if (grid.data[beginIndex+j+row_jumper] !=0 or lineNo != math.ceil((beginIndex+j)/grid.info.width-1.0)):	#check if this is end of line shomehow
					return False
				elif(j == 9):
					row_jumper = row_jumper + grid.info.width
			except IndexError:
				return False
	return True

def get_occupancy_value(coordinates,grid):
	if coordinates:
		x = int(round((coordinates[0] - grid.info.origin.position.x)/grid.info.resolution))
		y = int(round((coordinates[1] - grid.info.origin.position.y)/grid.info.resolution))
		index = x+y*grid.info.width
		return cell_clear(index,grid)
"""
def filter_path(path):
	normalizedPath = [path[0]]
	rotationPoint = path[0]
	way = ""
	for i in range(len(path)):
		if rotationPoint[0] == path[i+1][0] and way == "horizontal":
			pass
		elif rotationPoint[1] == path[i+1][1] and way == "vertical":
			pass
		elif rotationPoint[0]-1 == path[i+1][0] and way == "diagonalUpLeft":
			pass
		elif: rotationPoint[0]+1 == path[i+1][0] and way == "diagonalUpRight":
			pass
		elif: rotationPoint[0] == path[i+1][0] and way == "diagonalDownLeft":
			pass
		elif: rotationPoint[0] == path[i+1][0] and way == "diagonalDownRight":
			pass
		else:
			normalizedPath.append(path[i-1])
			rotationPoint = path[i-1]
	return normalizedPath
"""