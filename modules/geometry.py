#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
"""

import sys
import warnings

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from . import DrawingUtils

warnings.filterwarnings("ignore")

class GeometryUtils(object):
	"""
	"""

	def __init__(self):
		"""Constructor."""
		pass


	def __del__(self):
		"""Destructor."""
		del self


	def read_coordinates(self):
		# read the line as a string
		line = str(input()).rstrip()

		# read each line element with the map function
		# and making a list with it
		point = list(map(float, line.split()))
		return np.array(point)


	def make_vector(self, end, start):
		i = end[0] - start[0]
		j = end[1] - start[1]
		k = end[2] - start[2]

		return np.array([i, j, k])


	def process_input(self):
		try:
			# A, B, C (triangle points)
			A = self.read_coordinates()
			B = self.read_coordinates()
			C = self.read_coordinates()

			# instantiate vectors with the points
			AB = self.make_vector(B, A)
			AC = self.make_vector(C, A)
			BC = self.make_vector(C, B)
			CB = self.make_vector(B, C)
			CA = self.make_vector(A, C)

			# launch point
			L = self.read_coordinates()

			# amount of launches
			launches = int(input())

			# director vectors - throws
			throws = []
			for i in range(launches):
				throw = self.read_coordinates()
				throws.append(throw)

			self.check_collinearity(A, B, C)
			
			# check if the points form a triangle
			self.check_linear_independency(AB, AC)

			# check if the launch point lies on the target's plane
			self.check_launch_point(A, B, C, L)

		except Exception as e:
			print()
			print(e)
			sys.exit()

		points = [A, B, C, L]
		vectors = [AB, AC, BC, CB, CA]

		return points, vectors, launches, throws


	def plot(self, points, proportional_triangle, G, throw):
		A, B, C, L = points
		D, E, F = proportional_triangle

		# create the plot itself
		fig = plt.figure()
		ax = fig.gca(projection='3d')
		ax.set_aspect("equal")

		# Drawing Utils object
		drawing_utils = DrawingUtils()
		
		# set the limits
		ax = drawing_utils.set_limits(points, ax)

		# draw the triangle
		ax = drawing_utils.draw_vector_triangle(A, B, C, ax)

		# draw the small triangle
		ax = drawing_utils.draw_vector_triangle(D, E, F, ax)

		# draw G and L
		ax = drawing_utils.draw_point(G, 'g',ax)
		ax = drawing_utils.draw_point(L, 'r',ax)

		# draw shot
		ax = drawing_utils.draw_vector(L, L + throw, 'b',ax)

		plt.show()

	def check_collinearity(self, A, B, C):
		# (y2 − y1)(z3 − z1) − (y3 − y1)(z2 − z1) = 0
		# (x3 − x1)(z2 − z1) − (x2 − x1)(z3 − z1) = 0
		# (x2 − x1)(y3 − y1) − (x3 − x1)(y2 − y1) = 0

		a = (B[1] - A[1]) * (C[2] - A[2]) - (C[1] - A[1]) * (B[2] - A[2])
		b = (C[0] - A[0]) * (B[2] - A[2]) - (B[0] - A[0]) * (C[2] - A[2])
		c = (B[1] - A[1]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1])

		if a == 0 and b == 0 and c == 0:
			raise Exception("The points are collinear.")

		return


	def check_linear_independency(self, AB, AC):
		dot = np.dot(AB, AC)
		
		norm_ab = np.linalg.norm(AB)
		norm_ac = np.linalg.norm(AC)

		if norm_ab == 0 or norm_ac == 0:
			raise Exception("Vectors are equal.")

		cosine = dot / (norm_ab * norm_ac)

		if int(cosine) == 1 or int(cosine) == -1:
			raise Exception("Vectors are linearly dependent.")

		return


	def check_launch_point(self, p1, p2, p3, L):
		arr = np.vstack((np.array([p1, p2, p3, L]).T, [1, 1, 1, 1]))

		if np.linalg.det(arr) == 0:
			raise Exception("The launch point lies on the target's plane.")

		return


	def compute_centroid(self, A, B, C):

		x = 1 / 3 * (A[0] + B[0] + C[0])
		y = 1 / 3 * (A[1] + B[1] + C[1])
		z = 1 / 3 * (A[2] + B[2] + C[2])

		return np.array([x, y, z])


	def compute_proportional_triangle(self, A, B, C, G):
		d = np.linalg.norm((A, G))
		dt = d / 2
		t = dt / d

		x_coord = lambda x0, x1: (1 - t) * x0 + t * x1
		y_coord = lambda y0, y1: (1 - t) * y0 + t * y1
		z_coord = lambda z0, z1: (1 - t) * z0 + t * z1

		D = [x_coord(A[0], G[0]), y_coord(A[1], G[0]), z_coord(A[2], G[0])]
		E = [x_coord(B[0], G[0]), y_coord(B[1], G[0]), z_coord(B[2], G[0])]
		F = [x_coord(C[0], G[0]), y_coord(C[1], G[0]), z_coord(C[2], G[0])]

		proportional_triangle = [np.array(D), np.array(E), np.array(F)]

		return proportional_triangle


	def validate_throw_direction(self, AB, AC, L, w):
		ans = True

		# retrieve the normal vector of the plane
		N = np.cross(AB, AC)

		# definition of the throw vector
		TL = self.make_vector(w + L, L)

		# dot product of the throw vector and the normal vector
		dot = np.dot(TL, N)

		# cross product of the throw vector and the normal vector
		cross = np.cross(TL, N)

		# if the norm of the cross product is zero, they are collinear
		if np.linalg.norm(cross) == 0:
			ans = False

		# if the dot product is greater than zero, they have the same direction
		# therefore, it was thrown backwards
		if dot >= 0:
			ans = False
		
		return ans
		

	def intersection_point(self, AB, AC, L, G, w):

		# retrieve the normal vector of the plane
		N = np.cross(AB, AC)

		LG = self.make_vector(L, G)

		LG_dot_N = -np.dot(LG, N)

		w_dot_N = np.dot(w + L, N)

		tmp = (LG_dot_N / w_dot_N) * w

		intersection = L + tmp

		return intersection
		

	def throw_score(self, points, vectors, proportional_triangle, throw, G):
		AB, AC, BC, CB, CA = vectors
		D, E, F = proportional_triangle
		A, B, C, L = points

		# check if the launch was effective
		if not self.validate_throw_direction(AB, AC, L, throw):
			return -1
		
		# find the intersection point with the plane
		intersection = self.intersection_point(AB, AC, L, G, throw)

		# check which was the score region
		if np.array_equal(np.round(intersection, 2), np.round(G, 2)):
			return 20

		# retrieve the counterclockwise oriented normal vector of the plane
		N = np.cross(AB, AC)

		# vectors with the triangle coordinates and the intersection point
		AT = self.make_vector(intersection, A)
		BT = self.make_vector(intersection, B)
		CT = self.make_vector(intersection, C)
		
		# vectors with the smaller triangle coordinates and the intersection point
		DT = self.make_vector(intersection, D)
		ET = self.make_vector(intersection, E)
		FT = self.make_vector(intersection, F)

		# vectors of the smaller triangle
		DE = self.make_vector(E, D)
		DF = self.make_vector(F, D)
		FD = self.make_vector(D, F)
		EF = self.make_vector(F, E)

		triple = lambda a, b, c: np.dot(np.cross(b, c), a)

		# if the triple product of AB, AT and N is positive, it means that the
		# point T is to the left of the vector.
		if triple(DE, DT, N) > 0 and triple(EF, ET, N) > 0 and triple(FD, FT, N) > 0:
			return 10

		if triple(AB, AT, N) > 0 and triple(BC, BT, N) > 0 and triple(CA, CT, N) > 0:
			return 5
		
		# the throw was not successful, did not hit the target
		return 0
