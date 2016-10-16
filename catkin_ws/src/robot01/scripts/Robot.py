# http://wiki.ros.org/rospy/Overview/Parameter%20Server


#!/usr/bin/env python
import rospy
import roslib
import math
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from random import randint
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

############################################################################
# begining of class
############################################################################

class Robot:
	def __init__(self, dimensions_xyz=[]):
		rospy.init_node('simplebot')
		rospy.loginfo("Starting the robot")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.pubMarker = rospy.Publisher('/markerPoints', Marker, queue_size=100)
		rospy.Subscriber('/base_scan', LaserScan, self.detectObstacle)
		self.tf = tf.TransformListener()
		self.laserData = LaserScan()

		if(dimensions_xyz == []):
			self.dimensions_xyz = self.getDimensionsParam()
		else:
			self.dimensions_xyz = self.setDimensionsParam(dimensions_xyz)

		self.robotPose = RobotPoseBr(self.dimensions_xyz)
		self.update_rate = rospy.Rate(5)
		self.min_range = 0.5
		self.blocked = False

	def setDimensionsParam(self, dimensions_xyz):
		if (len(dimensions_xyz) == 3):
			rospy.set_param_raw('/robot/dimensions_xyz', [float(dimensions_xyz[0]),float(dimensions_xyz[1]),float(dimensions_xyz[2])])
			self.dimensions_xyz = rospy.get_param_raw('/robot/dimensions_xyz', [float(dimensions_xyz[0]),float(dimensions_xyz[1]),float(dimensions_xyz[2])])
		else:
			rospy.set_param_raw('/robot/dimensions_xyz', [1.0,1.0,0.25])

	def getDimensionsParam(self):
		if rospy.has_param('/robot/dimensions_xyz'):
			self.dimensions_xyz = rospy.get_param_raw('/robot/dimensions_xyz', [1.0,1.0,0.25])
		else:
			rospy.set_param_raw('/robot/dimensions_xyz', [1.0,1.0,0.25])
			self.dimensions_xyz = rospy.get_param_raw('/robot/dimensions_xyz', [1.0,1.0,0.25])
		
	def detectObstacle(self,laserScan):
		print "I am in detectObstacle"
		self.laserData = laserScan
		curAngle=laserScan.angle_min
		inc=laserScan.angle_increment
		self.blocked = False
		for range in laserScan.ranges:
			x=range*math.cos(curAngle)
			y=range*math.sin(curAngle)
			if ((abs(y)<=0.5) and (x<=self.min_range)):
				self.blocked = True
				print "DETECTED OBSTACLE!!!!!!!!!!!!!"
				break
			curAngle=curAngle+inc
	
	def getPosition(self):
		return self.tf.lookupTransform('/odom', '/base_link', rospy.Time(0))

	def drive(self):
		self.setRvizMarker(0, 0, 0.5, 0, 1, "/base_link")
		print "I am in drive"
		if (not self.blocked):
			try:
				curPosition = self.getPosition()
				print curPosition
				print curPosition[0]
			except(tf.LookupException):
				rospy.loginfo("LookupException")
				return
			except(tf.ConnectivityException):
				rospy.loginfo("ConnectivityException")
				return
			except(tf.ExtrapolationException):
				rospy.loginfo("ExtrapolationException")
				return
			wantedPoint = [2,1]
			if (curPosition[0] != wantedPoint[0]):

				self.cmd_vel.angular.z = 0
				self.cmd_vel.linear.x = 0.1
			elif(curPosition.point.y != wantedPoint[1]):
				self.cmd_vel.angular.z = 0
				self.cmd_vel.linear.x = 0.1

		else:
			self.cmd_vel.angular.z=randint(1, 9)
			self.cmd_vel.linear.x=0

		self.ctl_vel.publish(self.cmd_vel)	
		

############################################################################
# end of class
############################################################################

if __name__ == '__main__':
	try:
		robot = simplebot()
		while not rospy.is_shutdown():
			robot.drive()
			robot.update_rate.sleep()
		rospy.spin()

	except rospy.ROSInterruptException:
		print "Job Done. Nothing to do here."
		pass

#############################################################################