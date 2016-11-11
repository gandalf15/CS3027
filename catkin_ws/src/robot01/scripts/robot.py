#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
import tf
import math
from Queue import PriorityQueue
import astar
import controller
import marker

class Robot:
	"""docstring for Robot"""
	def __init__(self):
		rospy.init_node("Robot01")
		rospy.loginfo("Waiting for message /base_pose_ground_truth")
		rospy.wait_for_message("/base_pose_ground_truth", Odometry)
		rospy.loginfo("/base_pose_ground_truth ready")
		self.currentRealRobotPose = self.get_start()
		self.previousRealRobotPose = self.get_start()
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.setRealPose)
		self.floatGoals = self.get_goals()
		self.grid = self.get_map()
		goalsQueue = PriorityQueue()
		for g in self.floatGoals:
			if type(g[0]) == type(0.0) or type(g[1]) == type(0.0):
				goalsQueue.put((0,(round(g[0]),round(g[1]))))
			else:
				goalsQueue.put((0,(g[0],g[1])))
		self.prioritizedGoals = astar.prioritize_goals((self.currentRealRobotPose[0],self.currentRealRobotPose[1]), goalsQueue)
		rospy.loginfo("Goals were prioritized based on heuristic.")
		rospy.loginfo(self.prioritizedGoals)
		self.control = controller.Controller(self.get_start())
		self.pathMarkers = marker.Markers(rgbColour=[0,0.8,0], namespace="Path",frame="/map",markerSize_xyz=[1.0,1.0,0.01])
		self.realPathMarkers = marker.Markers(rgbColour=[1,0,0], namespace="realPath",frame="/map",markerSize_xyz=[0.5,0.5,0.4])
		self.goalMarkers = marker.Markers(rgbColour=[1,1,0], namespace="Goals",frame="/map",markerSize_xyz=[0.2,0.2,1.0])
		self.reachedGoalsMarkers = marker.Markers(rgbColour=[0.8,0.2,1], namespace="ReachedGoals",frame="/map",markerSize_xyz=[1.0,1.0,0.6])
		nextStart = (self.currentRealRobotPose[0],self.currentRealRobotPose[1])
		currentGoal = []
		for goal in self.floatGoals:	#draw markers for goals
			self.goalMarkers.add_marker(goal)
		for goal in self.prioritizedGoals:	#for every goal find path if possible and navigate there
			rospy.loginfo("Searching path to the next goal.")
			currentPath = astar.find_path(nextStart, goal, self.grid)
			if currentPath:
				rospy.loginfo("Path found.")
				for pose in currentPath:
					self.pathMarkers.add_marker(pose)
				floatGoalIndex = 0
				for i in range(len(self.floatGoals)):	#also store the value of precise position of goal for the last steps
					fGoal = self.floatGoals[i]
					if round(fGoal[0]) == goal[0] and round(fGoal[1]) == goal[1]:
						floatGoalIndex = i
						break
				self.control.set_path(currentPath, self.floatGoals[floatGoalIndex])
				nextStart = (currentPath[-1][0],currentPath[-1][1])
				self.pathMarkers.draw_markers()
				self.goalMarkers.draw_markers()
				rate = rospy.Rate(100)
				while not rospy.is_shutdown() and self.control.path:
					self.control.drive()
					rate.sleep()	#on my PC rviz frezes if I do not include some pause
					self.pathMarkers.draw_markers()
					rate.sleep()
					self.goalMarkers.draw_markers()
					if (abs(self.previousRealRobotPose[0]-self.currentRealRobotPose[0])>0.5 or \
					abs(self.previousRealRobotPose[1]-self.currentRealRobotPose[1])>0.5):	#draw real path markers only every 0.5 meter
						self.realPathMarkers.add_marker([self.currentRealRobotPose[0],self.currentRealRobotPose[1]])
						self.previousRealRobotPose = self.currentRealRobotPose
					rate.sleep()
					self.realPathMarkers.draw_markers()
					if self.control.goalReached == True:	# if controller set the value to true then draw marker
						self.reachedGoalsMarkers.add_marker([self.currentRealRobotPose[0],self.currentRealRobotPose[1]])
					self.reachedGoalsMarkers.draw_markers()
					rate.sleep()
		rospy.spin()
	
	def get_map(self):
		rospy.wait_for_service('static_map')
		try:
			map_storage = rospy.ServiceProxy('static_map', GetMap)
			return map_storage().map
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	def get_start(self):
		start = rospy.get_param("/robot_start", [0.0,0.0,0.0])
		return start

	def get_goals(self):
		points = []
		point = []
		for i in range(1,7):
			paramName = "/p%s"%str(i)
			point = rospy.get_param(paramName, [0.0,0.0])
			points.append(point)
		print points
		return points

	def setRealPose(self,data):
		self.currentRealRobotPose = [data.pose.pose.position.x, data.pose.pose.position.y]

try:
	robot01 = Robot()

except KeyboardInterrupt:
	print "Exiting"
	pass