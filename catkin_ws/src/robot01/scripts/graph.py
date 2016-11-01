#!/usr/bin/env python

"""
	inspiration: 
	http://www.python-course.eu/graphs_python.php
	http://www.redblobgames.com/pathfinding/grids/graphs.html
	http://www.bogotobogo.com/python/python_graph_data_structures.php
"""

class Node:
	"""docstring for node"""
	def __init__(self, pose):
		self.position = pose
		self.neighbors = {}

	def __str__(self):
		return str(self.position) + ' neighbors: ' + str([node.position for node in self.neighbors])

	def add_neighbour(self, node, weight=1):
		self.neighbors[node] = weight

	def get_position(self):
		return self.position

	def get_weight(self, neighbor):
		return self.neighbors[neighbor]

	def get_neighbors(self):
		return self.neighbors.keys()

class Graph:
	"""docstring for Graph"""
	def __init__(self):
		self.nodes = {}
		self.nodesCount = 0

	def __iter__(self):
		return iter(self.nodes.values())

	def add_node(self, pose):
		self.nodesCount = self.nodesCount + 1
		newNode = Node(pose)
		self.nodes[pose] = newNode
		return newNode

	def get_node(self, pose):
		if pose in self.nodes:
			return self.nodes[node]
		else:
			return None

	def add_edge(self, frm, to, cost=1):
		if frm not in self.nodes:
			self.add_node(frm)
		if to not in self.nodes:
			self.add_node(to)

		self.nodes[frm].add_neighbour(self.nodes[to], cost)
		self.nodes[to].add_neighbour(self.nodes[frm], cost)

	def get_nodes(self):
		return self.nodes.keys()
