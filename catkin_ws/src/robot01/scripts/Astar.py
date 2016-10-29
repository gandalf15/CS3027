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
		
class Astar:
	"""docstring for Astar"""
	def __init__(self,start_point_xy, goal_points_xy, grid):
		self.grid = grid
		self.startPose = (round(start_point_xy[0]),round(start_point_xy[1]))
		poseQueue = PriorityQueue()
		for i in range(len(goal_points_xy)): 
			poseQueue.put((0,(round(goal_points_xy[i][0]),round(goal_points_xy[i][1]))))
		self.prioritizedGoals = []
		self.__prioritize_goals(self.startPose, poseQueue)
		self.robot_dimensions_xyz = rospy.get_param('/robot/dimensions_xyz', [1.0,1.0,0.25])
		self.unvisitedQueue = PriorityQueue()
		self.path = []
		startPose = self.startPose
		for nextPose in self.prioritizedGoals:
			self.find_path(startPose, nextPose)
			startPose = nextPose
		print self.path

	def __prioritize_goals(self, start_pose, goal_queue):
		remainingPose = PriorityQueue()
		if goal_queue.qsize() > 1:
			for i in range(goal_queue.qsize()):
				pose = goal_queue.get()[1]
				priority = self.heur(pose,start_pose)
				remainingPose.put((priority,pose))
			closestPose = remainingPose.get()
			self.prioritizedGoals.append(closestPose[1])
			self.__prioritize_goals(closestPose[1],remainingPose)
		else:
			lastPose = goal_queue.get()[1]
			self.prioritizedGoals.append(lastPose)

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
			return self.__check_cell(index)

	def __check_cell(self,index):
		row_jumper = 0
		beginIndex = (index-5)-self.grid.info.width*5
		lineNo = math.ceil(beginIndex/self.grid.info.width-1.0)
		for i in range(10):
			for j in range(10):
				try:
					#print "new line number: ",math.ceil((beginIndex+j)/self.grid.info.width-1.0)
					if (self.grid.data[beginIndex+j+row_jumper] !=0 or lineNo != math.ceil((beginIndex+j)/self.grid.info.width-1.0)):	#check if this is end of line shomehow
						return 100
					elif(j == 9):
						row_jumper = row_jumper + self.grid.info.width
				except IndexError:
					return 1
		return 0


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
