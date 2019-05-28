#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""""""

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import numpy as np

class DrawingUtils(object):
	"""
	"""

	def __init__(self):
		"""Constructor."""
		pass


	def __del__(self):
		"""Destructor."""
		del self


	def draw_vector_triangle(self, A, B, C, ax):
		"""Draws 3 tridimensional vectors to form a triangle.

		:param A, B, C
			Points in a 3D space.

		:param ax
			Axes object to draw in.
		
		:return                 
			The axes object with a vectorial triangle.
		"""
		# AB vector
		ab = Arrow3D([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], mutation_scale=10,
					lw=1, arrowstyle="-|>", color="k")

		# BC vector
		bc = Arrow3D([B[0], C[0]], [B[1], C[1]], [B[2], C[2]], mutation_scale=10,
					lw=1, arrowstyle="-|>", color="k")

		# CA vector
		ca = Arrow3D([C[0], A[0]], [C[1], A[1]], [C[2], A[2]], mutation_scale=10,
					lw=1, arrowstyle="-|>", color="k")

		# pack all the vector in the Axes instance.
		ax.add_artist(ab)
		ax.add_artist(bc)
		ax.add_artist(ca)

		return ax


	def draw_point(self, P, color, ax):
		"""Draws a point in a 3D space.

		:param P
			The point to be drawn.

		:param ax
			Axes object to draw in.
		
		:return                 
			The axes object with a point.
		"""
		ax.scatter(P[0], P[1], P[2], color=color, s=100)
		return ax


	def set_limits(self, points, ax):
		"""Sets the limits of an Axes instance.

		The maximum limit will be set as the maximum from all
		the possible coordinates to be drawn.

		:param points
			A list of all possible points to be drawn in the space.

		:param ax
			Axes object to draw in.
		
		:return                 
			The axes object with new limits set.
		"""
		m = np.max(points)

		ax.set_xlim(-m, m)
		ax.set_ylim(-m, m)
		ax.set_zlim(-m, m)

		return ax


	def draw_vector(self, P, Q, color, ax):
		"""Draws a vector in a 3D space.

		:param P
			The origin of the vector.

		:param P
			The directional vector.

		:param ax
			Axes object to draw in.
		
		:return                 
			The axes object with a 3D vector.
		"""
		v = Arrow3D([P[0], Q[0]], [P[1], Q[1]], [P[2], Q[2]], mutation_scale=10,
				lw=1, arrowstyle="-|>", color=color)
		
		ax.add_artist(v)
		return ax

	
class Arrow3D(FancyArrowPatch):
	"""Module for drawing a 3D arrow.

	Adapted from the SasView package.
	https://github.com/SasView
	"""

	def __init__(self, xs, ys, zs, *args, **kwargs):
		"""Constructor.
			
		:Params xs: [[x0, x0+dx0], [x1, x1+dx1], ...]
		:Params ys: [[y0, y0+dy0], [y1, y1+dy1], ...]
		:Params zs: [[z0, z0+dz0], [z1, z1+dz1], ...]
		"""
		FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
		self._verts3d = xs, ys, zs
		
		return

	def draw(self, renderer):
		"""Binding with the add_artist method to draw the content."""

		xs3d, ys3d, zs3d = self._verts3d
		xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
		self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
		FancyArrowPatch.draw(self, renderer)

		return
		