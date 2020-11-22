import numpy as np
import sympy as sp
from Entity import Line, Point
from Geometry import Geometry
from abc import ABC
from utils.coordination_calculator import *

# GunTurret only functions: setP0T, trajectoryPath, fwdkin, inverseKin
# CameraTurret only functions: cameraPlaneBounds, getTargetLinks, representTarget, sweepPath
class Turret(Geometry, ABC):
	def __init__(self, q1, q2, pOffset, p12, orig):
		super().__init__()
		self.q1_given = q1
		self.q2_given = q2
		self.pOffset = pOffset
		self.p12 = p12
		self.orig = orig

		self.q1, self.q2 = sp.symbols('q1 q2')

	def setQ1(self, q1):
		self.q1_given = q1

	def setQ2(self, q2):
		self.q2_given = q2

	def getEntities(self):
		pass

	# TODO: Maybe have one function getPath that would call other path functions based on some input string

	"""
    WIP: S Curve based on position distance instead of angular distance using Jacobian
    def scurvePath(self, init_pos, dest_pos, coeff_s, t_elapse, time_step):
        # Find unit vector and norm from init_pos to dest_pos
        norm = np.linalg.norm(dest_pos - init_pos)
        u = (dest_pos - init_pos)/norm

        # Determine S-Curve coefficients
        # S(t) = A/(1 + e^(-k(t - t0)))
        # A = norm
        # t0 = 1/2 target time to destination
        # k = coeff_s <--- TODO
        t0 = t_elapse/2

        # Find number of time steps and allocate path matrix at size
        # q x s, where q is the number of joints and s is the number of time steps
        num_steps = math.ceil(t_elapse/time_step)
        omega = np.zeroes([self.num_q, num_steps])

        for i in range(num_steps):
            # Find current distance along unit vector
            s = norm/(1 + math.e**(-coeff_s((i*time_step) - t0)))
            s_prime = s(1 - s)

            # Project along unit vector
            p = s_prime * u

            # Convert to angular velocities with jacobian
    """
	def scurvePath(self, init_q, dest_q, coeff_s, t_elapse, time_step):
		# Find unit vector and norm from init_pos to dest_pos
		#norm = np.linalg.norm(dest_q - init_q)
		norm = ((dest_q[0, 0] - init_q[0, 0])**2 + (dest_q[1, 0] - init_q[1, 0])**2)**.5

		if norm == 0:
		    return np.array([[0, dest_q[0][0], dest_q[1][0]]]).T

		u = (dest_q - init_q) / norm

		# Determine S-Curve constants
		# S(t) = A/(1 + e^(-k(t - t0)))
		# A = norm
		# t0 = 1/2 target time to destination
		# k = coeff_s <--- TODO
		t0 = t_elapse / 2

		# Find number of time steps and allocate path matrix at size
		# q x s, where q is the number of joints and s is the number of time steps
		num_steps = np.ceil(t_elapse / time_step)
		q_steps = np.zeros([self.num_q+1, num_steps])

		for i in range(num_steps):
		    # Find current distance along unit vector
		    s = norm / (1 + math.e ** (-coeff_s * ((i * time_step) - t0)))

		    # Project along unit vector and add initial q
		    q_step = s * u + init_q

		    q_steps[0, i:i+1] = (i * time_step)
		    q_steps[1:3,i:i+1] = q_step

		return q_steps
