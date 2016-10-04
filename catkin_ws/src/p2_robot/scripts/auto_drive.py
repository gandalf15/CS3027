#!/usr/bin/env python
import rospy
import roslib
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class simplebot:
    def __init__(self):
    	rospy.init_node('simplebot')
    	self.min_range = 0.5
    	self.width = 0.35
    	self.subscriber = rospy.Subscriber('/base_scan', LaserScan, self.checkDistance)
    	self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    	self.rate = rospy.Rate(5)

    def checkDistance(self, laserScan):
    	curAngle = laserScan.angle_min
    	inc = laserScan.angle_increment
    	tw=Twist()
    	obstacle = False
    	for range in laserScan.ranges:
    		x = range * math.cos(curAngle)
    		y = range * math.sin(curAngle)

    		if ((abs(y)<self.width/1.5) and (x<self.min_range)):
    			obstacle = True
    			print "Obstacle at ", x, " ", y, " be careful!"

    		curAngle = curAngle + inc

    	if (not obstacle):
    		print "I am here"
    		tw.angular.z = 0
    		tw.linear.x = 0.5
    	else:
    		tw.angular.z=1
    		tw.linear.x=0
    	print tw
    	self.publisher.publish(tw)

if __name__ == '__main__':
	try:
		robot = simplebot()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass