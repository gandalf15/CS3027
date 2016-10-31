#!/usr/bin/env python

"""
	# http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
	# http://www.redblobgames.com/pathfinding/a-star/introduction.html

"""
import math
import rospy
from sys import maxint
from Queue import PriorityQueue
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
		
class Astar:
	"""docstring for Astar"""
	def __init__(self,start_point_xy, goal_points_xy, grid):
		rospy.init_node('Astar')
		rospy.loginfo("Starting A star")
		self.markerPathPub = rospy.Publisher('/AstarPath', MarkerArray, queue_size=100)
		self.grid = grid
		self.startPose = None
		self.unvisitedQueue = PriorityQueue()
		self.path = []
		self.prioritizedGoals = []
		self.markerAry = []
		self.markerId = 0
		self.reset_path(start_point_xy,goal_points_xy)

	def reset_path(self, start_point_xy, goal_points_xy):
		self.path = []
		self.prioritizedGoals = []
		self.startPose = (round(start_point_xy[0]),round(start_point_xy[1]))
		poseQueue = PriorityQueue()
		for i in range(len(goal_points_xy)): 
			poseQueue.put((0,(round(goal_points_xy[i][0]),round(goal_points_xy[i][1]))))
		self.__prioritize_goals(self.startPose, poseQueue)
		startPose = self.startPose
		for nextPose in self.prioritizedGoals:
			self.find_path(startPose, nextPose)
			startPose = nextPose

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
		self.clear_markers()
		while not unvisitedQueue.empty():
			currentPose = unvisitedQueue.get()[1]
			if currentPose == goal:
				while currentPose != start:
					path.insert(0, [currentPose[0],currentPose[1]])
					currentPose = parents[currentPose]
				self.path.append(path)
				print self.path
				for i in range(len(self.path)):
					for j in self.path[i]:
						self.__addMarker(j,1,0.1,0.1,"AstarPath","/map")
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

	def __addMarker(self,position,r,g,b,namespace,frame_id,):
		marker = Marker()
		marker.header.frame_id = frame_id
		marker.ns = namespace
		marker.id = self.markerId
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.pose.position.x = position[0]
		marker.pose.position.y = position[1]
		marker.pose.orientation.w = 1
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
		marker.color.r = r
		marker.color.g = g
		marker.color.b = b
		marker.color.a = 1
		self.markerAry.append(marker)
		self.markerId += 1

	def draw_markers(self):
		Array = MarkerArray()
		for m in self.markerAry:
			Array.markers.append(m)
		self.markerPathPub.publish(Array)
		self.markerId = 0

	def clear_markers(self):
		Array=MarkerArray()
		for m in self.markerAry:
			m.action = m.DELETE
			Array.markers.append(m)
		self.markerPathPub.publish(Array)
		self.markerId = 0
		self.markerAry = []