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
		rospy.Subscriber('/base_scan', LaserScan, self.checkDistance)
		self.tf = tf.TransformListener()
		self.update_rate = rospy.Rate(5)
		self.min_range = 0.5
		self.width = 0.35

	def transformToBase(self,x,y):
		try:
			ps=PointStamped(header=Header(stamp=rospy.Time.now(),frame_id="/base_laser_link"), point=Point(x,y,0))

			#print listener.transformPoint("/base_link",ps), "\n", Point(x,y,0)
			#print "\n"
			return self.tf.transformPoint('/base_link', ps)
		except(tf.LookupException):
			print "LookupException"
			return
		except(tf.ConnectivityException):
			print "ConnectivityException"
			return
		except(tf.ExtrapolationException):
			print "ExtrapolationException"
			return

	def checkDistance(self, laserScan):
		curAngle=laserScan.angle_min
		inc=laserScan.angle_increment
		self.obstacle = False
		for range in laserScan.ranges:
			x=range*math.cos(curAngle)
			y=range*math.sin(curAngle)
			#np = self.transformToBase(x, y)
			ps=PointStamped(header=Header(stamp=rospy.Time.now(),frame_id="/base_laser_link"), point=Point(x,y,0))
			np=self.tf.transformPoint('/base_link', ps)
			new_x = np.point.x
			new_y = np.point.y
			curAngle=curAngle+inc

	def detectObstacle(self, baseLaserScan):
		for range in baseLaserScan.ranges:
			
		if ((abs(new_y)<self.width) and (new_x<self.min_range)):
				#print "obstacle at \nX: ", new_x, "\n", "Y: ", new_y, "\n"
				self.cmd_vel = self.set_vel(0)
		#curPosition = self.tf.lookupTransform('/base_link', '/odom', rospy.Time(0))
		if (not self.obstacle):
			self.cmd_vel.angular.z = 0
			self.cmd_vel.linear.x = 0.5
		else:
			self.cmd_vel.angular.z=5
			self.cmd_vel.linear.x=0
		print tw
		self.ctl_vel.publish(self.cmd_vel)
		#print curPosition
		return

############################################################################
# end of class
############################################################################

if __name__ == '__main__':
	try:
		robot = simplebot()
		robot.update_rate.sleep()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

#############################################################################