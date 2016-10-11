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
		self.min_range = 0.5
		self.width = 0.35
		self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.checkDistance)
		self.tf_subscriber = tf.TransformListener()
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.obstacle = False

	def transformToBase(self,x,y):
		try:
			ps=PointStamped(header=Header(stamp=rospy.Time.now(),frame_id="/base_laser_link"), point=Point(x,y,0))

			#print listener.transformPoint("/base_link",ps), "\n", Point(x,y,0)
			#print "\n"
			return self.tf_subscriber.transformPoint('/base_link', ps)
		except(tf.LookupException):
			return
		except(tf.ConnectivityException):
			return
		except(tf.ExtrapolationException):
			return

	def checkDistance(self, laserScan):
		curAngle=laserScan.angle_min
		inc=laserScan.angle_increment
		self.obstacle = False
		for range in laserScan.ranges:
			x=range*math.cos(curAngle)
			y=range*math.sin(curAngle)
			np = self.transformToBase(x, y)
			if np:
				new_x = np.point.x
				new_y = np.point.y

				if ((abs(new_y)<self.width) and (new_x<self.min_range)):
					#print "obstacle at \nX: ", new_x, "\n", "Y: ", new_y, "\n"
					self.obstacle = True

			curAngle=curAngle+inc

		tw=Twist()
		curPosition = self.tf_subscriber.lookupTransform('/base_link', '/odom', rospy.Time(0))
		if (not self.obstacle):
			tw.angular.z = 0
			tw.linear.x = 0.5
		else:
			tw.angular.z=5
			tw.linear.x=0
		print tw
		self.publisher.publish(tw)
		print curPosition
		return

############################################################################
# end of class
############################################################################

if __name__ == '__main__':
	try:
		robot = simplebot()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

#############################################################################