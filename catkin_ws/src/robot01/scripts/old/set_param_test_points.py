#!/usr/bin/env python
import rospy
from random import randint

rospy.init_node('set_param_test_points')

def set_param_points(points=[0]):
	if (points != [0]):
		i = 1
		for p in points:
			paramName = "/p%s"%str(i)
			paramVal = str(p)
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
	infoMessage = "Points were set: %s"%str(aryPoints)
	rospy.loginfo(infoMessage)

set_param_points()