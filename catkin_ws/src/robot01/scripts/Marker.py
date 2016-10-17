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
	def __init__(self,namespace="default_name",frame="default_frame",markerType=Marker.SPHERE,markerSize_xyz=[0.2,0.2,0.2]):
		super(ClassName, self).__init__()
		self.id = 0
		self.namespace = namespace
		self.frame_id = frame
		self.type = markerType
		self.scale = Vector3(x=markerSize_xyz[0],y=markerSize_xyz[1],z=markerSize_xyz[2])
		self.markers = []
		self.publisher = rospy.Publisher(self.namespace, MarkerArray, queue_size=10)

	def add_marker(self):
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.ns = self.namespace
		marker.id = self.id
		self.id += 1
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.2

	def set_position(self,x,y):

	def set_colour(self,r,g,b)

print Marker.ARROW
print Vector3(x=1,y=2,z=0.5)