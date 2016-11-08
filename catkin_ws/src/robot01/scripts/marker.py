#!/usr/bin/env python

"""
	# http://wiki.ros.org/rviz/DisplayTypes/Marker
	# http://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
"""

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3

# taken from practical 4 and extended
class Markers:
	"""docstring for Markers"""
	def __init__(self,rgbColour=[1,0,0], namespace="name",frame="frame",markerSize_xyz=[0.2,0.2,0.2]):
		#rospy.init_node("Markers")
		self.markerId = 0
		self.namespace = namespace
		self.frameId = frame
		self.type = Marker.CUBE
		self.scale = Vector3(x=markerSize_xyz[0],y=markerSize_xyz[1],z=markerSize_xyz[2])
		self.markers = []
		self.markerPub = rospy.Publisher(self.namespace, MarkerArray, queue_size=10)
		self.rgbColour = rgbColour

	def add_marker(self, position_xy, ):
		marker = Marker()
		marker.header.frame_id = self.frameId
		marker.ns = self.namespace
		marker.id = self.markerId
		marker.type = self.type
		marker.action = marker.ADD
		marker.pose.position.x = position_xy[0]
		marker.pose.position.y = position_xy[1]
		marker.pose.orientation.w = 1
		marker.scale = self.scale
		marker.color.r = self.rgbColour[0]
		marker.color.g = self.rgbColour[1]
		marker.color.b = self.rgbColour[2]
		marker.color.a = 1
		self.markers.append(marker)
		self.markerId += 1

	def draw_markers(self):
		Array = MarkerArray()
		for m in self.markers:
			Array.markers.append(m)
		self.markerPub.publish(Array)

	def clean_markers(self):
		Array=MarkerArray()
		if self.markers:
			for m in self.markers:
				m.action = m.DELETE
				Array.markers.append(m)
		self.markerPub.publish(Array)
		self.markerId = 0
		self.markerAry = []