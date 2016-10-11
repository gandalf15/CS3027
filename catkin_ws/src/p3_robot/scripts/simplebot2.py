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

##############################################################################

class simplebot:
	def __init__(self):
		rospy.init_node('simplebot')
		rospy.loginfo("Starting the robot")
		self.ctl_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.cmd_vel = Twist()
		self.aryLaserBaseData = LaserScan()
		rospy.Subscriber('/base_scan', LaserScan, self.detectObstacle)
		self.tf = tf.TransformListener()
		self.laserData = LaserScan()
		self.update_rate = rospy.Rate(10)
		self.min_range = 0.5
		self.width = 0.35
		self.blocked = False

	def detectObstacle(self,laserScan):
		print "I am in transformLaserToBase"
		self.laserData = laserScan
		curAngle=laserScan.angle_min
		inc=laserScan.angle_increment
		self.blocked = False
		for range in laserScan.ranges:
			x=range*math.cos(curAngle)
			y=range*math.sin(curAngle)
			if ((abs(y)<=0.25-0.05) and (x<=self.min_range)):
				self.blocked = True
				break
			curAngle=curAngle+inc
			
	def drive(self):
		print "I am in drive"
		if (not self.blocked):
			self.cmd_vel.angular.z = 0
			self.cmd_vel.linear.x = 1
		else:
			self.cmd_vel.angular.z=randint(1, 9)
			self.cmd_vel.linear.x=0

		self.ctl_vel.publish(self.cmd_vel)	
		

	def getPosition(self):
		return self.tf.lookupTransform('/base_link', '/odom', rospy.Time(0))

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