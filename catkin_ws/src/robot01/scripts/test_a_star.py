#!/usr/bin/env python

#import rospy
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray

from nav_msgs.srv import GetMap
from Astar import*
#from RobotPoseBr import*
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def addMarker(position,r,g,b,namespace,frame_id,marker_id,markerAry):
	marker = Marker()
	marker.header.frame_id = frame_id
	marker.ns = namespace
	marker.id = marker_id
	marker.type = marker.CUBE
	marker.action = marker.ADD
	marker.pose.position.x = position[0]
	marker.pose.position.y = position[1]
	marker.pose.orientation.w = 1
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.r = r
	marker.color.g = g
	marker.color.b = b
	marker.color.a = 1
	markerAry.append(marker)
	return markerAry
	#self.pubMarker.publish(self.markerAry)

def get_map():
	rospy.wait_for_service('static_map')
	try:
		map_storage = rospy.ServiceProxy('static_map', GetMap)
		return map_storage()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def draw_markers(markerPub, markerAry):
	Array = MarkerArray()
	for m in markerAry:
		Array.markers.append(m)
	markerPub.publish(Array)

try:
	rospy.init_node('test')
	print "starting"
	#test = MapSub()
	#rospy.sleep(5)
	print "decomposition is done"
	#robotBr = RobotPoseBr()
	#print test.grid
	
	astar = Astar(start_point_xy = [-64.00,0.00], goal_points_xy = [[-1.0,37.0],[-52.0,.0]], grid = get_map().map)

	markerPathPub = rospy.Publisher('/AstarPath', MarkerArray, queue_size=100)
	#broadcaster = tf.TransformBroadcaster()
	markerAry = []
	marker_id = 0
	for i in range(len(astar.path)):
		for j in astar.path[i]:
			markerAry = addMarker(j,0.1,1,1,"lol","/map",marker_id,markerAry)
			marker_id += 1
	while not rospy.is_shutdown():
		draw_markers(markerPathPub, markerAry)
		rospy.sleep(2)
	rospy.spin()
except KeyboardInterrupt:
	print "Exiting"
	pass