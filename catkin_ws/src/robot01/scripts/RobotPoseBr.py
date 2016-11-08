#!/usr/bin/env python

"""
	# http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20(Python)
	# http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20(Python)
	# http://wiki.ros.org/PyStyleGuide
"""

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class RobotPoseBr:
	"""docstring for ClassName"""
	def __init__(self, dimensions_xyz=[1,1,0.25]):
		rospy.init_node('Robot_Pose_Broadcaster')
		rospy.loginfo("Starting the Robot_Pose_Broadcaster")
		self.realPose = Odometry()
		self.amclPose = PoseWithCovarianceStamped()
		self.dimensions_xyz = dimensions_xyz
		self.broadcaster = tf.TransformBroadcaster()
		self.markerPub = rospy.Publisher('/RobotPoseMarker', MarkerArray, queue_size=1)
		self.markerAry = []
		self.marker_id = 0
		self.updateRate = rospy.Rate(10)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.handle_real_position)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.handle_amcl_position)
		while not rospy.is_shutdown():
			self.broadcast_position(self.realPose)
			self.set_real_pose_marker()
			self.set_amcl_pose_marker()
			self.draw_markers()
			self.updateRate.sleep()
			self.clean_markers()
		rospy.spin()
			
	def addMarker(self,position,orientation,r,g,b,namespace,frame_id,alpha):
		marker = Marker()
		marker.header.frame_id = frame_id
		marker.ns = namespace
		marker.id = self.marker_id
		self.marker_id += 1
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.pose.position = position
		marker.pose.orientation = orientation
		marker.scale.x = self.dimensions_xyz[0]
		marker.scale.y = self.dimensions_xyz[1]
		marker.scale.z = self.dimensions_xyz[2]
		marker.color.r = r
		marker.color.g = g
		marker.color.b = b
		marker.color.a = alpha
		self.markerAry.append(marker)
		#self.pubMarker.publish(self.markerAry)

	def draw_markers(self):
		markerArray = MarkerArray()
		for m in self.markerAry:
			markerArray.markers.append(m)
		self.markerPub.publish(markerArray)
		self.marker_id = 0
		self.markerAry = []

	def clean_markers(self):
		markerArray=MarkerArray()
		for m in self.markerAry:
			m.action = m.DELETE
			markerArray.markers.append(m)
		self.markerPub.publish(markerArray)

	def broadcast_position(self,odometryData):
		l = self.realPose.pose.pose.position
		q = self.realPose.pose.pose.orientation
		try:
			self.broadcaster.sendTransform((l.x, l.y,l.z), 
											(q.x, q.y, q.z, q.w), 
											rospy.Time.now(), 
											'/real_robot_pose', 
											'/map')
		except:
			rospy.loginfo("Broadcast transform real_robot_pose EXCEPTION!")
	
	def set_amcl_pose_marker(self):
		position = self.amclPose.pose.pose.position
		orientation = self.amclPose.pose.pose.orientation
		namespace = 'robot_amcl_pose'
		frame_id = '/map'
		self.addMarker(position,orientation,0.1,0.1,1,namespace,frame_id,alpha=1)
	def set_real_pose_marker(self):
		position = self.realPose.pose.pose.position
		orientation = self.realPose.pose.pose.orientation
		namespace = 'robot_real_pose'
		frame_id = '/map'
		self.addMarker(position,orientation,0.1,0.1,0.1,namespace,frame_id,alpha=1)

	def handle_amcl_position(self,amclData):
		self.amclPose = amclData

	def handle_real_position(self,odometryData):
		self.realPose = odometryData

robotBr = RobotPoseBr()