#!/usr/bin/env python
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class markerPlacer:
	def __init__(self):
		rospy.init_node('markerPlacer')
		self.id=0
		self.markArray = MarkerArray()
		self.count=0
		self.MARKER_MAX=100
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.transform)
		self.publisher = rospy.Publisher("/scanMarkers", MarkerArray, queue_size=100, latch=True)
		self.br = tf.TransformBroadcaster()
		self.listener = tf.TransformListener()
		rospy.spin()
		
	#Does transformations
	def transform(self, odomMsg):
		p=odomMsg.pose.pose.position
		o=odomMsg.pose.pose.orientation
		try:
			t = self.listener.getLatestCommonTime("/map", "/real_robot_pose")
			self.br.sendTransform((p.x,p.y,p.z), (o.x, o.y, o.z, o.w), t, "/real_robot_pose", "/map")
			self.addMarkers(odomMsg)
		except:
			pass
		
	#will eventually add marker to that spot
	def addMarkers(self, trnsfrm):
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.ns = 'robotTruePose'
		marker.id = self.id
		self.id += 1
		self.count += 1
		marker.type = marker.ARROW
		marker.action = marker.ADD
		marker.scale.x = 1.2
		marker.scale.y = 1.2
		marker.scale.z = 1.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation = trnsfrm.pose.pose.orientation
		marker.pose.position = trnsfrm.pose.pose.position
		
		# We add the new marker to the MarkerArray, removing the oldest
		# marker from it when necessary
		if(self.count > self.MARKER_MAX):
			self.markArray.markers.pop(0)
			self.count-=1
		self.markArray.markers.append(marker)
		# now you can call the function whenever you want.
		# this means that you can extend your code and add more markers and then publish them at one time
		# you just have to call this function after you add markers
		self.publishMarkers()
		
	def publishMarkers(self):
		# Publish the MarkerArray
		print "adding marker!!!"
		self.publisher.publish(self.markArray)
		self.id = 0
		self.count = 0
		
		rospy.sleep(0.1)
robot=markerPlacer()