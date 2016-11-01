#!/usr/bin/env python

import numpy as np
import copy as cp

def grid_to_matrix(data, width, height):
	matrix = np.ndarray((height, width), np.bool)
	print matrix
	print len(matrix)
	print len(matrix[0])
	for row in range(height):
		for col in range(width):
			if data[row*width + col] != 0:
				matrix[row][col] = True 	#obstacle
			else:
				matrix[row][col] = False 	#clean
	return matrix

def expand_occupancy_matrix(matrix,expand_value = 1):
	expandedMatrix = cp.deepcopy(matrix)
	width = len(matrix[0])
	height = len(matrix)
	for row in range(height):
		for col in range(width):
			if matrix[row][col] == True:
				for expRow in range(expand_value*2+1):
					for expCol in range(expand_value*2+1):
						curRow = row-expand_value+expRow
						curCol = col-expand_value+expCol
						if curRow >= 0 and curRow < height:
							if curCol >= 0 and curCol < width:
								expandedMatrix[row-expand_value+expRow][col-expand_value+expCol] = True
	return expandedMatrix