#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Main module for receiving parameters and guide the process."""

import warnings
import modules

def main():
	"""
	"""
	geometry_utils = modules.GeometryUtils()

	# validate input parameters and unpack all of them
	points, vectors, launches, throws = geometry_utils.process_input()

	# compute the centroid of the triangle
	G = geometry_utils.compute_centroid(points[0], points[1], points[2])

	# compute the coordinates of the smaller triangle
	proportional_triangle = geometry_utils.compute_proportional_triangle(points[0], points[1], points[2], G)

	score = 0
	
	for throw in throws:
		tmp = geometry_utils.throw_score(points, vectors, proportional_triangle, throw, G)
		score += tmp
		
		geometry_utils.plot(points, proportional_triangle, G, throw)

	print('Score: {}'.format(score))

	return


if __name__ == '__main__':
	main()
