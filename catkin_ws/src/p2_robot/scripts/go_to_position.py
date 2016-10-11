#!/usr/bin/env python
import rospy
import roslib
import math
from random import randint
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class simplebot:
    def __init__(self):
    	rospy.init_node('simplebot')
    	self.min_range = 0.5
    	self.width = 0.35
    	#self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.checkDistance)
        self.odometry_subscriber = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.get_current_position)
    	self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    	self.rate = rospy.Rate(5)

    def get_current_position(self, odometry):
        currtent_position = odometry.pose.pose.position
        wanted_position_x = -15
        wanted_position_y = 22 
        if (currtent_position.x != wanted_position_x):
            
        else:
            

    def checkDistance(self, laserScan):
    	curAngle = laserScan.angle_min
    	inc = laserScan.angle_increment
        print inc
        print len(laserScan.ranges)
    	tw=Twist()
    	obstacle = False
    	for range in laserScan.ranges:
    		x = range * math.cos(curAngle)
    		y = range * math.sin(curAngle)

    		if ((abs(y)<self.width/2) and (x<self.min_range)):
    			obstacle = True
    			print "Obstacle at ", x, " ", y, " be careful!"

    		curAngle = curAngle + inc

    	if (not obstacle):
    		tw.angular.z = 0
    		tw.linear.x = 0.5
    	else:
    		tw.angular.z=randint(-9, 9)
    		tw.linear.x=0
    	print tw
    	self.publisher.publish(tw)

if __name__ == '__main__':
	try:
		robot = simplebot()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass