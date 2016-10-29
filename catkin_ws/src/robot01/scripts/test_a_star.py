#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap
import Astar
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Controller:
	"""docstring for Controller"""
	def __init__(self, wanted_pose):
		rospy.init_node("Controller")
		rospy.loginfo("Starting Controller")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.laserData = LaserScan()
		self.dimensions_xyz = self.setDimensionsParam(dimensions_xyz)
		self.wantedPose = wanted_pose
		self.currentPose = Odometry()
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.currentPose)
		rospy.Subscriber('/base_scan', LaserScan, self.detectObstacle)
		
	def get_map():
		rospy.wait_for_service('static_map')
		try:
			map_storage = rospy.ServiceProxy('static_map', GetMap)
			return map_storage()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setDimensionsParam(self, dimensions_xyz):
		if (len(dimensions_xyz) == 3):
			rospy.set_param('/robot/dimensions_xyz', [float(dimensions_xyz[0]),float(dimensions_xyz[1]),float(dimensions_xyz[2])])
			self.dimensions_xyz = rospy.get_param('/robot/dimensions_xyz', [float(dimensions_xyz[0]),float(dimensions_xyz[1]),float(dimensions_xyz[2])])
		else:
			rospy.set_param('/robot/dimensions_xyz', [1.0,1.0,0.25])

	def detectObstacle(self,laserScan):
		print "I am in detectObstacle"
		self.laserData = laserScan
		curAngle=laserScan.angle_min
		inc=laserScan.angle_increment
		self.blocked = False
		for range in laserScan.ranges:
			x=range*math.cos(curAngle)
			y=range*math.sin(curAngle)
			if ((abs(y)<=1.5) and (x<=self.min_range*2)):
				self.blocked = True
				print "DETECTED OBSTACLE!!!!!!!!!!!!!"
				break
			curAngle=curAngle+inc
	
	def get_current_pose(self):
		x = self.currentPose.pose.pose.position.x
		y = self.currentPose.pose.pose.position.y
		return [x,y]

	def drive(self):
		print "I am in drive"
		if (not self.blocked):
			if (self.get_current_pose() != self.wantedPose):
				self.cmd_vel.angular.z = 0
				self.cmd_vel.linear.x = 0.5
		else:
			self.cmd_vel.angular.z=0.5
			self.cmd_vel.linear.x=1

		self.ctl_vel.publish(self.cmd_vel)	




try:
	rospy.Subscriber('/real_robot_pose', Odometry, handle_real_position)
	#print real_pose
	astar = Astar.Astar(start_point_xy = [-64.00,0.00], goal_points_xy = [[52.64,-21.78],[-1.0,37.0],[-52.0,0.0]], grid = get_map().map)
	

	rospy.spin()
except KeyboardInterrupt:
	print "Exiting"
	pass