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

	
def get_map():
	rospy.wait_for_service('static_map')
	try:
		map_storage = rospy.ServiceProxy('static_map', GetMap)
		return map_storage()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


class Controller:
	"""docstring for Controller"""
	def __init__(self, goalPose):
		rospy.init_node("Controller2")
		rospy.loginfo("Starting Controller")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.laserData = LaserScan()
		self.dimensions_xyz = self.setDimensionsParam()
		self.wantedPose = goalPose
		self.odomData = Odometry()
		self.goalPose = goalPose
		self.goalStamped = PointStamped()
		self.goalTheta = 0.0
		self.tf = tf.TransformListener()
		#self.pose = Pose()
		self.lastOdomTime = 0

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
		try:
			ps = PointStamped()
			ps.point.x = self.goalPose[0]
			ps.point.y = self.goalPose[1]
			ps.header.stamp = self.tf.getLatestCommonTime("/map","/base_link")
			ps.header.frame_id = "/map"
			self.goalStamped = self.tf.transformPoint("/base_link", ps)
			self.goalTheta = math.atan2(self.goalStamped.point.y, self.goalStamped.point.x)
			print self.goalTheta
		except:
			pass


	def drive(self):
		print "I am in drive"
		if (not self.blocked):
			if (self.get_current_pose() != self.wantedPose):
				self.cmd_vel.angular.z = 0
				self.cmd_vel.linear.x = 0.5
		else:
			self.cmd_vel.angular.z=1
			self.cmd_vel.linear.x=0

		self.ctl_vel.publish(self.cmd_vel)	




try:
	#print real_pose
	#astar = Astar.Astar(start_point_xy = [-64.00,0.00], goal_points_xy = [[52.64,-21.78],[-1.0,37.0],[-52.0,0.0]], grid = get_map().map)
	#astar.draw_markers()
	#goal = astar.path[0][0]
	#print goal
	control = Controller((-63.0,0.0))

	rospy.spin()
except KeyboardInterrupt:
	print "Exiting"
	pass