#!/usr/bin/env python
from random import randint
import rospy
#rospy.init_node('set_param_test_points')

def set_param_points(points=[0]):
	if (points != [0]):
		i = 1
		aryPoints = points
		for p in points:
			paramName = "/p%s"%str(i)
			paramVal = p#str(p)
			rospy.set_param(paramName, paramVal)
			i += 1
	else:
		i = 1
		aryPoints = []
		for n in range(0, 6):
			point=[]
			point.append(randint(-65,65))
			point.append(randint(-65,65))
			aryPoints.append(point)
		for p in aryPoints:
			paramName = "/p%s"%str(i)
			paramVal = p#str(p)
			rospy.set_param(paramName, paramVal)
			i += 1

	rospy.set_param("/start_pos", [-64.0,0.0])

	infoMessage = "Points were set: %s"%str(aryPoints)
	rospy.loginfo(infoMessage)

set_param_points([[4.30,36.83],[54.28,26.39],[-21.60,-39.51],[-32.72,3.93],[60.6,45.76],[-0.59,-27.56]])