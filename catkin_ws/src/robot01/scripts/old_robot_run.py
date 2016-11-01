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
	def __init__(self):
		rospy.init_node('simplebot')
		rospy.loginfo("Starting the robot")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.pubMarker = rospy.Publisher('/markerPoints', Marker, queue_size=100)
		self.cmd_vel = Twist()
		self.aryLaserBaseData = LaserScan()
		self.markerAry=MarkerArray()
		rospy.Subscriber('/base_scan', LaserScan, self.detectObstacle)
		self.tf = tf.TransformListener()
		self.laserData = LaserScan()
		self.update_rate = rospy.Rate(5)
		self.min_range = 0.5
		self.width = 0.35
		self.blocked = False

	def setRvizMarker(self,x,y,r,g,b,frame):
		mr=Marker()
		mr.header.frame_id=frame
		mr.ns="robot"
		mr.id=1
		mr.type=mr.CUBE
		mr.action=mr.ADD
		mr.pose.position.x=x
		mr.pose.position.y=y
		mr.pose.orientation.w=1
		mr.scale.x=0.35
		mr.scale.y=0.35
		mr.scale.z=0.5
		mr.color.r=r
		mr.color.g=g
		mr.color.b=b
		mr.color.a=1.0
		self.pubMarker.publish(mr)
		print self.markerAry

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