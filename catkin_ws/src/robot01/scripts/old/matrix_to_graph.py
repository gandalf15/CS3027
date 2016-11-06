#!/usr/bin/env python

import graph as G

def matrix_to_graph(matrix):
	graphOfMap = G.Graph()
	height = len(matrix) 
	width = len(matrix[0])
	for row in range(height):
		print "row no: ", row
		for col in range(width):
			if matrix[row][col] == False:
				for newRow in range(3):
					for newCol in range(3):
						neighbourRow = row-1+newRow
						neighbourCol = col-1+newCol
						if neighbourRow >= 0 and neighbourRow < height:
							if neighbourCol >= 0 and neighbourCol < width:
								if matrix[neighbourRow][neighbourCol] == False:
									if (col,row) != (neighbourCol,neighbourRow):
										graphOfMap.add_edge((col,row), (neighbourCol,neighbourRow)) 	# swapped cols with rows to represent (x,y) coords in map
	return graphOfMap