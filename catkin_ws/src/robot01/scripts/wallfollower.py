#!/usr/bin/python
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

LEFT=1
RIGHT=0

class Wallfollower:
	def __init__(self):
		rospy.init_node('wallfollower')
		rospy.wait_for_service('global_localization')
		glob_local = rospy.ServiceProxy('global_localization', Empty)
		glob_local()
		self.safeDistance=2
		self.following=LEFT
		#followingwallonLEFTorRIGHT
		self.listener=tf.TransformListener()
		self.publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.subscriber=rospy.Subscriber('/base_scan',LaserScan,self.scanReceived)
	def scanReceived(self,LaserScan):
		a=LaserScan.angle_min
		i=LaserScan.angle_increment
#initialiseanglesforscanningtoleftorrightoftherobot
#dependingonwallfollowingdirection.
		if (self.following==RIGHT):
			start=-999
			end=0
		else:
			start=0
			end=999
#setrangestoarbitrarynumbersinitially
		rmin=9999
		amin=999
		for r in LaserScan.ranges:#findclosestpoint
			if(a<=end and a>=start and r<rmin):
				amin=a
				rmin=r
			a+=i
#computepointinlaserframeofreference
#andtransformtorobotframe
		x=rmin*math.cos(amin)
		y=rmin*math.sin(amin)
		ps=PointStamped(header=Header(stamp=rospy.Time(0),frame_id="/base_laser_link"),point=Point(x,y,0))
		p=self.listener.transformPoint("/base_link",ps)
		x=p.point.x
		y=p.point.y
		mag = rmin #math.sqrt(x*x+y*y)
#usefulforlater,magnitudeofvector
#tangentvector
		tx=0
		ty=0
		if(self.following==RIGHT):
			tx=-y
			ty=x
		else:
			tx=y
			ty=-x
		tx/=mag
		ty/=mag
		#where we should be going
		dx=x+tx-self.safeDistance*x/mag
		dy=y+ty-self.safeDistance*y/mag
		#self.m.add(dx,dy,0,0,1.0,"/map")

		theta=0

		if (rmin<LaserScan.range_max):
			theta=math.atan2(dy,dx)
		elif (self.following==RIGHT):
			theta=-1
		else:
			theta=1

		twist=Twist()
		if (dx>0.05):
			twist.linear.x=0.5
		twist.angular.z=theta
		self.publisher.publish(twist)
		#self.m.add(0,0,1.0,0,0,"/map")
		#self.m.draw()
		#self.m.clear()

robot=Wallfollower()
rospy.spin()


