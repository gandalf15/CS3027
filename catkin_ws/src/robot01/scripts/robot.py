#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
import math
from Queue import PriorityQueue
import astar
import controller
import marker
import set_param_test_points as SetPoints

class Robot:
	"""docstring for Robot"""
	def __init__(self):
		rospy.init_node("Robot01")
		rospy.loginfo("Waiting for message /base_pose_ground_truth")
		rospy.wait_for_message("/base_pose_ground_truth", Odometry)
		rospy.loginfo("/base_pose_ground_truth ready")
		self.realRobotPose = []
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.setRealPose)
		self.floatGoals = self.get_goals()
		self.startPose = self.get_start()
		self.grid = self.get_map()
		goalsQueue = PriorityQueue()
		for g in self.floatGoals:
			if type(g[0]) == type(0.0) or type(g[1]) == type(0.0):
				goalsQueue.put((0,(round(g[0]),round(g[1]))))
			else:
				goalsQueue.put((0,(g[0],g[1])))
		self.prioritizedGoals = astar.prioritize_goals(self.startPose, goalsQueue)
		rospy.loginfo("Goals were prioritized based on heuristic.")
		rospy.loginfo(self.prioritizedGoals)
		self.control = controller.Controller()
		self.pathMarkers = marker.Markers(rgbColour=[0,0.5,0], namespace="Path",frame="/map",markerSize_xyz=[1.0,1.0,0.01])
		self.realPathMarkers = marker.Markers(rgbColour=[0,0.5,0], namespace="realPath",frame="/map",markerSize_xyz=[0.5,0.5,0.05])
		self.goalMarkers = marker.Markers(rgbColour=[1,1.5,0], namespace="Goals",frame="/map",markerSize_xyz=[0.2,0.2,1.0])
		nextStart = self.startPose
		currentGoal = []
		for goal in self.prioritizedGoals:
			self.goalMarkers.add_marker(goal)
		for goal in self.prioritizedGoals:
			currentPath = astar.find_path(nextStart, goal, self.grid)
			if currentPath:
				for pose in currentPath:
					self.pathMarkers.add_marker(pose)
				floatGoalIndex = 0
				for i in range(len(self.floatGoals)):
					fGoal = self.floatGoals[i]
					if round(fGoal[0]) == goal[0] and round(fGoal[1]) == goal[1]:
						floatGoalIndex = i
						break
				self.control.set_path(currentPath, self.floatGoals[floatGoalIndex])
				nextStart = (currentPath[-1][0],currentPath[-1][1])
				self.pathMarkers.draw_markers()
				self.goalMarkers.draw_markers()
				rate = rospy.Rate(10)
				while not rospy.is_shutdown() and self.control.path:
					self.control.drive()
					self.pathMarkers.draw_markers()
					self.goalMarkers.draw_markers()
					self.realPathMarkers.add_marker(self.realRobotPose)
					self.realPathMarkers.draw_markers()
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
		start = rospy.get_param("/start_pos", [0.0,0.0])
		start = (round(start[0]),round(start[1]))
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
		self.realRobotPose = [data.pose.pose.position.x, data.pose.pose.position.y]

try:
	robot01 = Robot()

except KeyboardInterrupt:
	print "Exiting"
	pass