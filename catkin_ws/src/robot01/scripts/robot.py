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
	
def get_map():
	rospy.wait_for_service('static_map')
	try:
		map_storage = rospy.ServiceProxy('static_map', GetMap)
		return map_storage().map
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_goals():
	points = []
	point = []
	for i in range(1,7):
		paramName = "/p%s"%str(i)
		point = rospy.get_param(paramName, [0.0,0.0])
		points.append(point)
	print points
	return points

try:
	rospy.init_node("Robot01")
	print"call set param"
	SetPoints.set_param_points([[-2.42,35.20],[-60.0,2.34],[27.67,28.87],[6.53,51.47],[-6.67,68.87],[-7.27,60.47]])
	print "points are set"	
	goals = get_goals()
	startPose = (-64.0,0.0)
	grid = get_map()
	goalsQueue = PriorityQueue()
	for g in goals:
		if type(g[0]) == type(0.0) or type(g[1]) == type(0.0):
			goalsQueue.put((0,(round(g[0]),round(g[1]))))
		else:
			goalsQueue.put((0,(g[0],g[1])))
	prioritizedGoals = astar.prioritize_goals(startPose, goalsQueue)
	print prioritizedGoals
	controll = controller.Controller()
	pathMarkers = marker.Markers(rgbColour=[1,0,0], namespace="Path",frame="/map",markerSize_xyz=[1.0,1.0,1.0])
	for goal in prioritizedGoals:
		print goal
		path = astar.find_path(startPose, goal, grid)
		print path
		if path:
			for pose in path:
				pathMarkers.add_marker(pose)
			controll.set_path(path)
			startPose = (path[-1][0],path[-1][1])
			rate = rospy.Rate(50)
			while not rospy.is_shutdown() and controll.path:
				controll.drive()
				pathMarkers.draw_markers()
				rate.sleep()


	rospy.spin()
except KeyboardInterrupt:
	print "Exiting"
	pass