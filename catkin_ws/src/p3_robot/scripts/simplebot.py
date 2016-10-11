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

##############################################################################

class simplebot:
	def __init__(self):
		rospy.init_node('simplebot')
		rospy.loginfo("Starting the robot")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.aryLaserBaseData = []
		rospy.Subscriber('/base_scan', LaserScan, self.transformLaserToBase)
		self.tf = tf.TransformListener()
		self.update_rate = rospy.Rate(10)
		self.min_range = 0.5
		self.width = 0.35
		self.blocked = False		


	def transformPointToBase(self, x, y, strFrameId):
		try:
			ps=PointStamped(header=Header(stamp=rospy.Time(0),frame_id=strFrameId), point=Point(x,y,0))
			return self.tf.transformPoint('/base_link', ps)
		except(tf.LookupException):
			rospy.loginfo("LookupException")
			return
		except(tf.ConnectivityException):
			rospy.loginfo("ConnectivityException")
			return
		except(tf.ExtrapolationException):
			rospy.loginfo("ExtrapolationException")
			return

	def transformLaserToBase(self,laserScan):
		print "I am in transformLaserToBase"
		self.aryLaserBaseData=[]
		curAngle=laserScan.angle_min
		inc=laserScan.angle_increment
		for range in laserScan.ranges:
			x=range*math.cos(curAngle)
			y=range*math.sin(curAngle)
			np=self.transformPointToBase(x,y,"/base_laser_link")
			while not np:
				print "in while loop"
				np=self.transformPointToBase(x,y,"/base_laser_link")
			print "Point X: ", x, "\nPoint y: ", y
			print "New point: ", np
			self.aryLaserBaseData.append(np)
			curAngle=curAngle+inc
			
	# check if there is an obstacle in front
	def detectObstacle(self):
		print "I am in detectObstacle"
		for range in self.aryLaserBaseData:
			print "Point X: ", range.point.x, "\nPoint y: ", range.point.y
			if ((abs(range.point.y)<=0.25) and (range.point.x<=self.min_range)):
				return True
		return False

	def drive(self):
		print "I am in drive"
		if self.aryLaserBaseData != []:
			self.blocked = False
			self.blocked=self.detectObstacle()
			if (not self.blocked):
				self.cmd_vel.angular.z = 0
				self.cmd_vel.linear.x = 0.5
			else:
				self.cmd_vel.angular.z=0.5
				self.cmd_vel.linear.x=0

			self.ctl_vel.publish(self.cmd_vel)
		else:
			print "empty array"
			self.cmd_vel.angular.z=0
			self.cmd_vel.linear.x=0
			self.ctl_vel.publish(self.cmd_vel)


	def getPosition(self):
		return self.tf.lookupTransform('/base_link', '/odom', rospy.Time(0))

############################################################################
# end of class
############################################################################

if __name__ == '__main__':
	robot = simplebot()
	while not rospy.is_shutdown():	
		try:
			robot.drive()
			robot.update_rate.sleep()
			rospy.spin()
		except rospy.ROSInterruptException:
			print "Job Done. Nothing to do here."
			pass

#############################################################################