# http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20(Python)
# http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20(Python)
# http://wiki.ros.org/PyStyleGuide

import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class RobotPoseBr:
	"""docstring for ClassName"""
	def __init__(self):
		self.realPose = Odometry()
		self.odomPose = Odometry()
		self.broadcaster = tf.TransformBroadcaster()
		self.markerPub = rospy.Publisher('/RealRobotPoseMarker', MarkerArray, queue_size=100)
		self.markerAry = []
		self.updateRate = rospy.Rate(10)
		self.markerCounter = 0
		rospy.init_node('Real_Robot_Pose_Broadcaster')
		rospy.loginfo("Starting the Real_Robot_Pose_Broadcaster")
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.handlePosition)
		while not rospy.is_shutdown():
			self.broadcastPosition(self.realPose)
			self.drawPositionReal(self.realPose)
			#self.drawPositionOdom()
			self.updateRate.sleep()
			
	def setRvizMarker(self,x,y,r,g,b,frame,nameSpace):
		marker = Marker()
		marker.header.frame_id = frame
		marker.ns = nameSpace
		marker.id = self.markerCount
		self.markerCounter += 1
		marker.type=marker.CUBE
		marker.action=marker.ADD
		marker.pose.position.x=x
		marker.pose.position.y=y
		marker.pose.orientation.w=1
		marker.scale.x=0.35
		marker.scale.y=0.35
		marker.scale.z=0.5
		marker.color.r=r
		marker.color.g=g
		marker.color.b=b
		marker.color.a=1.0
		self.markerAry.append(marker)
		#self.pubMarker.publish(self.markerAry)

	def drawPositionReal(self,odometryData):


	def drawPositionOdom(self,odometryData):


	def broadcastPosition(self,odometryData):
		self.broadcaster.sendTransform(odometryData.pose.pose.position, 
										odometryData.pose.pose.orientation, 
										rospy.Time.now(), 
										'/real_robot_pose', 
										'/map')

	def handlePosition(self,odometryData):
		self.realPose = odometryData

