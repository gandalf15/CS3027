#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap
import astar
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
import math

class Controller:
	"""docstring for Controller"""
	def __init__(self):
		#rospy.init_node("Controller")
		#rospy.loginfo("Starting Controller")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.laserData = LaserScan()
		self.dimensions_xyz = self.setDimensionsParam()
		self.odomData = Odometry()
		self.path = None
		self.goalMapPose = None
		self.goalBasePose = [1.0,1.0]
		self.goalTheta = 0.0
		self.tf = tf.TransformListener()
		self.currentOdomPose = [0.0,0.0]
		self.currentMapPose = [0.0,0.0]
		self.lastOdomTime = 0
		self.blocked = False

		rospy.wait_for_message("/odom", Odometry)
		rospy.loginfo("odom ready")
		rospy.Subscriber("/odom", Odometry, self.get_odom)
		#rospy.Subscriber('/base_scan', LaserScan, self.detectObstacle)


	def set_path(self,path):
		self.path = path
		self.goalMapPose = self.path.pop(0)

	def setDimensionsParam(self):
		rospy.set_param('/robot/dimensions_xyz', [1.0,1.0,0.25])
		return [1.0,1.0,0.25]

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
	
	def get_odom(self, data):
		self.lastOdomTime = rospy.get_time()
		self.odomData = data
		self.currentOdomPose[0] = round(data.pose.pose.position.x)
		self.currentOdomPose[1] = round(data.pose.pose.position.y)

		try:
			ps = PointStamped()
			ps.point.x = self.goalMapPose[0]
			ps.point.y = self.goalMapPose[1]
			ps.header.stamp = self.tf.getLatestCommonTime("/map","/base_link")
			ps.header.frame_id = "/map"
			newPs = self.tf.transformPoint("/base_link", ps)
			self.goalBasePose = [newPs.point.x,newPs.point.y]
			self.goalTheta = math.atan2(newPs.point.y, newPs.point.x)
		except:
			pass

	def drive(self):
		if (not self.blocked):
			if (abs(self.goalBasePose[0])>0.4 or abs(self.goalBasePose[1])>0.4):
				self.cmd_vel.angular.z = self.goalTheta
				self.cmd_vel.linear.x = 0.5
			else:
				print "point reached"
				self.goalMapPose = self.path.pop(0)
				try:
					ps = PointStamped()
					ps.point.x = self.goalMapPose[0]
					ps.point.y = self.goalMapPose[1]
					ps.header.stamp = self.tf.getLatestCommonTime("/map","/base_link")
					ps.header.frame_id = "/map"
					newPs = self.tf.transformPoint("/base_link", ps)
					self.goalBasePose = [newPs.point.x,newPs.point.y]
					self.goalTheta = math.atan2(newPs.point.y, newPs.point.x)
				except:
					print "exception while TF point"
					pass
				self.cmd_vel.angular.z = 0.0
				self.cmd_vel.linear.x = 0.0
		else:
			self.cmd_vel.angular.z = 0.5
			self.cmd_vel.linear.x = 0.0

		self.ctl_vel.publish(self.cmd_vel)	