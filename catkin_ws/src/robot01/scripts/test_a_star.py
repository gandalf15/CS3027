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
from geometry_msgs.msg import Pose
import tf
import math
	
def get_map():
	rospy.wait_for_service('static_map')
	try:
		map_storage = rospy.ServiceProxy('static_map', GetMap)
		return map_storage()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


class Controller:
	"""docstring for Controller"""
	def __init__(self, path):
		rospy.init_node("Controller")
		rospy.loginfo("Starting Controller")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.laserData = LaserScan()
		self.dimensions_xyz = self.setDimensionsParam()
		self.odomData = Odometry()
		self.path = path
		self.goalPose = self.path.pop(0)
		self.goalStamped = PointStamped()
		self.goalTheta = 0.0
		self.tf = tf.TransformListener()
		self.currentOdomPose = [0.0,0.0]
		self.lastOdomTime = 0
		self.blocked = False

		rospy.wait_for_message("/odom", Odometry)
		rospy.loginfo("odom ready")
		rospy.Subscriber("/odom", Odometry, self.get_odom)
		#rospy.Subscriber('/base_scan', LaserScan, self.detectObstacle)

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
			ps.point.x = self.goalPose[0]
			ps.point.y = self.goalPose[1]
			ps.header.stamp = self.tf.getLatestCommonTime("/map","/base_link")
			ps.header.frame_id = "/map"
			self.goalStamped = self.tf.transformPoint("/base_link", ps)
			self.goalTheta = math.atan2(self.goalStamped.point.y, self.goalStamped.point.x)
		except:
			pass


	def drive(self):
		#print "I am in drive"
		if (not self.blocked):
			if (self.currentOdomPose != self.goalPose):
				self.cmd_vel.angular.z = self.goalTheta
				self.ctl_vel.publish(self.cmd_vel)
				self.cmd_vel.linear.x = 0.5
			else:
				print "point reached"
				self.goalPose = self.path.pop(0)
				self.cmd_vel.angular.z = 0.0
				self.cmd_vel.linear.x = 0.0
		else:
			self.cmd_vel.angular.z = 0.5
			self.cmd_vel.linear.x = 0.0

		self.ctl_vel.publish(self.cmd_vel)	




try:
	#astar = Astar.Astar(start_point_xy = [-64.00,0.00], goal_points_xy = [[52.64,-21.78],[-1.0,37.0],[-52.0,0.0]], grid = get_map().map)
	#astar.draw_markers()
	#goal = astar.path[0][0]
	path=[[-52.0, 1.0], [-52.0, 2.0], [-52.0, 3.0], [-52.0, 4.0], [-52.0, 5.0], [-52.0, 6.0], [-52.0, 7.0], [-52.0, 8.0], [-52.0, 9.0], [-52.0, 10.0], [-52.0, 11.0], [-52.0, 12.0], [-52.0, 13.0], [-52.0, 14.0], [-52.0, 15.0], [-52.0, 16.0], [-52.0, 17.0], [-52.0, 18.0], [-52.0, 19.0], [-52.0, 20.0], [-52.0, 21.0], [-52.0, 22.0], [-52.0, 23.0], [-52.0, 24.0], [-52.0, 25.0], [-52.0, 26.0], [-52.0, 27.0], [-52.0, 28.0], [-52.0, 29.0], [-52.0, 30.0], [-52.0, 31.0], [-52.0, 32.0], [-52.0, 33.0], [-52.0, 34.0], [-51.0, 34.0], [-50.0, 34.0], [-49.0, 34.0], [-48.0, 34.0], [-47.0, 34.0], [-46.0, 34.0], [-45.0, 34.0], [-44.0, 34.0], [-43.0, 34.0], [-42.0, 34.0], [-41.0, 34.0], [-40.0, 34.0], [-39.0, 34.0], [-38.0, 34.0], [-37.0, 34.0], [-36.0, 34.0], [-35.0, 34.0], [-34.0, 34.0], [-33.0, 34.0], [-32.0, 34.0], [-31.0, 34.0], [-30.0, 34.0], [-29.0, 34.0], [-28.0, 34.0], [-27.0, 34.0], [-26.0, 34.0], [-25.0, 34.0], [-24.0, 34.0], [-23.0, 34.0], [-22.0, 34.0], [-21.0, 34.0], [-20.0, 34.0], [-19.0, 34.0], [-18.0, 34.0], [-17.0, 34.0], [-16.0, 34.0], [-15.0, 34.0], [-14.0, 34.0], [-13.0, 34.0], [-12.0, 34.0], [-11.0, 34.0], [-10.0, 34.0], [-9.0, 34.0], [-8.0, 34.0], [-7.0, 34.0], [-6.0, 34.0], [-5.0, 34.0], [-5.0, 33.0], [-5.0, 32.0], [-5.0, 31.0], [-5.0, 30.0], [-5.0, 29.0], [-5.0, 28.0], [-5.0, 27.0], [-5.0, 26.0], [-5.0, 25.0], [-5.0, 24.0], [-5.0, 23.0], [-5.0, 22.0], [-5.0, 21.0], [-5.0, 20.0], [-5.0, 19.0], [-5.0, 18.0], [-5.0, 17.0], [-5.0, 16.0], [-5.0, 15.0], [-5.0, 14.0], [-5.0, 13.0], [-5.0, 12.0], [-4.0, 12.0], [-3.0, 12.0], [-3.0, 13.0], [-3.0, 14.0], [-3.0, 15.0], [-3.0, 16.0], [-3.0, 17.0], [-3.0, 18.0], [-3.0, 19.0], [-2.0, 19.0], [-1.0, 19.0], [0.0, 19.0], [1.0, 19.0], [2.0, 19.0], [2.0, 20.0], [2.0, 21.0], [1.0, 21.0], [0.0, 21.0], [-1.0, 21.0], [-1.0, 22.0], [-1.0, 23.0], [-1.0, 24.0], [-1.0, 25.0], [-1.0, 26.0], [-1.0, 27.0], [-1.0, 28.0], [-1.0, 29.0], [-2.0, 29.0], [-2.0, 30.0], [-3.0, 30.0], [-3.0, 31.0], [-3.0, 32.0], [-3.0, 33.0], [-3.0, 34.0], [-3.0, 35.0], [-3.0, 36.0], [-3.0, 37.0], [-2.0, 37.0], [-1.0, 37.0]]
	control = Controller(path)
	while not rospy.is_shutdown():
		control.drive()

	rospy.spin()
except KeyboardInterrupt:
	print "Exiting"
	pass