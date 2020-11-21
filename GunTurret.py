import numpy as np
from Entity import Line, Point
from Geometry import Geometry
from utils.coordination_calculator import *
import sympy as sp
import math
import matplotlib.pyplot as plt

class GunTurret(Geometry):
    def __init__(self, q1_given, q2_given, v0, p0T,
                 p01=np.array([0, -100, 0]).reshape(3, 1),
                 p12=np.array([0, 0, 75]).reshape(3, 1),
                 pOffset=np.array([0, 50, 0]).reshape(3, 1),
                 orig=np.array([100, 50, 0]).reshape(3, 1)):
        super().__init__()
        self.q1_given = q1_given
        self.q2_given = q2_given
        self.v0 = v0
        self.p01 = p01
        self.p12 = p12
        self.pOffset = pOffset
        self.orig = orig
        self.num_q = 2

        #  Target
        self.p0T = p0T

        # Trajectory calculations at angle
        self.q1, self.q2, self.t = sp.symbols('q1 q2 t')
        self.dv = -(1/2) * 9.81 * sp.cos(self.q2) * self.t**2
        self.dh = -(1/2) * 9.81 * sp.sin(self.q2) * self.t**2 + self.v0 * self.t

        # Derive forms of homogenous transform T and Jacobian J
        self.T, self.J = self.fwdkin()

    def setQ1(self, q1):
        self.q1_given = q1

    def setQ2(self, q2):
        self.q2_given = q2

    def setP0T(self, p0T):
        self.p0T = p0T

    def getEntities(self):
        # Two methods of displaying the gunturret (redundancy is intended):
        # 1. Run inverse kinematics and compute q1 and q2, then use computed f and toa to form a path.
        # 2. Plug in provided q1 and q2 into homogenous transform and find f. Apply pathing with some large toa.

        # Method 1
        #q1, q2, toa, f = self.inverseKin()

        # Method 2
        q1 = self.q1_given
        q2 = self.q2_given
        toa = 10
        f = self.T.subs([[self.q1, q1], [self.q2, q2]])[0:3, 3:4]

        R01 = zRot(q1)
        R12 = xRot(q2)

        p_1 = self.orig + self.p01 + R01 @ self.p12
        p_2 = p_1 + R01 @ R12 @ self.pOffset

        # q_init q_dest coeff_s time_elapsed time_step
        q_mat = self.scurvePath(np.array([0, 0]).reshape(2, 1),
                                np.array([q1, q2]).reshape(2, 1),
                                6, 1.5, .05)

        #plt.plot(q_mat[0:1,:], q_mat[1:2,:], 'ro', markersize=5)
        #plt.plot(q_mat[0:1,:], q_mat[2:3,:], 'bo', markersize=5)
        #plt.ylabel('Angle (rad)')
        #plt.xlabel('Timestep (s)')
        #plt.show()

        #self.T.subs([[self.q1, q1], [self.q2, q2], [self.t, toa]])
        #self.J.subs([[self.q1, q1], [self.q2, q2], [self.t, toa]])

        path_entities = self.trajectoryPath(toa, f, 30)

        return [Line(np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), p_2, "black", 2),
                Line(np.array([0, p_2[1][0], 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),
                Line(np.array([p_2[0][0], 0, 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),

                Line(self.orig + self.p01, p_1, "orange", 8),
                Line(p_1, p_2, "orange", 8),

                Point(self.orig + self.p01, 'black', 10),
                Point(p_1, 'black', 10),
                Point(p_2, 'black', 10),
                ] + path_entities

    def trajectoryPath(self, toa, f, steps):
        # Creates list of dot primitives to represent parabolic path
        interval = toa/steps
        entities = []
        for i in range(steps + 1):
            t = i * interval
            p = f.subs(self.t, t)
            np_p = self.orig + np.array(p).astype(np.float64)
            entities.append(Point(np_p, 'black', 2))
        return entities

    def fwdkin(self):
        # Forward kinematics with POE and iterative Jacobian form
        # TODO: Make h vectors an attribute of all geometries. Iterative method is broken up for now
        J = sp.zeros(6, self.num_q+1);

        T01 = sp.eye(4)
        T01[0:3, 0:3] = zRot_s(self.q1)
        T01[0:3, 3:4] = sp.Matrix(self.p01)

        J = phi(sp.Matrix(zRot_s(self.q1).T), sp.Matrix(self.p01)) * J
        J[0:6, 0:1] = sp.Matrix([0, 0, 1, 0, 0, 0])

        T12 = sp.eye(4)
        T12[0:3, 0:3] = xRot_s(self.q2)
        T12[0:3, 3:4] = sp.Matrix(self.p12)

        J = phi(sp.Matrix(xRot_s(self.q1).T), sp.Matrix(self.p12)) * J
        J[0:6, 1:2] = sp.Matrix([1, 0, 0, 0, 0, 0])

        T2T = sp.eye(4)
        p2T = sp.Matrix(self.pOffset) + sp.Matrix([[0], [self.dh], [self.dv]])
        T2T[0:3, 3:4] = p2T

        # Complex prismatic joint, not along axes
        J = phi(sp.eye(3), sp.Matrix(p2T)) * J
        # Find unit vector of p2T
        p2T_u = p2T/p2T.norm()
        # Insert p2T
        J[0:6, 2:3] = sp.Matrix([0, 0, 0, p2T_u])

        return T01 * T12 * T2T, J

    def inverseKin(self):
        # Arrange kinematic chain into subproblem 4
        # P0T = P01 + R12 @ P12 + R12 @ R2T @ P2T
        # ezT(P12 + P2T) = exT @ Rz(q1)T @ (P0T - P01)
        # Where P2T is pOffset + [0, dh, dv] = [0; l2 + dh; dv]
        exT = sp.Matrix([[1, 0, 0]])
        Rzq1 = sp.Matrix([[sp.cos(self.q1), -sp.sin(self.q1), 0],
                            [sp.sin(self.q1), sp.cos(self.q1), 0],
                            [0, 0, 1]])

        expr = exT * Rzq1 * (sp.Matrix(self.p0T) - sp.Matrix(self.p01))

        p2T = sp.Matrix(self.pOffset) + sp.Matrix([[0], [self.dh], [self.dv]])
        d = exT * (sp.Matrix(self.p12) + p2T)

        # Solve kinematic chain for q1. Only ever one solution.
        q1_sol = sp.solve(expr - d, self.q1)[0][0]

        # Substitute q1 into kinematic chain and solve for q2 using subproblem 3
        # POT - P01 - R12 @ P12 = R12 @ R2T @ P2T
        # R12T @ (POT - P01 - R12 @ P12) = R2T @ P2T
        # |R12T @ (POT - P01 - R12 @ P12)| = |R2T @ P2T|
        # lhs_norm = rhs_norm
        #
        # Since system has an extra t variable due to trajectory, use z relation

        lhs = Rzq1.T * (sp.Matrix(self.p0T) - sp.Matrix(self.p01) - Rzq1 * sp.Matrix(self.p12))
        lhs_val = lhs.subs(self.q1, q1_sol)

        lhs_norm = (lhs_val[0]**2 + lhs_val[1]**2 + lhs_val[2]**2)**.5
        lhs_z = lhs_val[2]

        Rxq2 = sp.Matrix([[1, 0, 0],
                         [0, sp.cos(self.q2), -sp.sin(self.q2)],
                         [0, sp.sin(self.q2), sp.cos(self.q2)]])

        rhs = Rxq2 * p2T
        rhs_norm = (rhs[0]**2 + rhs[1]**2 + rhs[2]**2)**.5
        rhs_z = rhs[2]

        esys = sp.Matrix([[lhs_norm - rhs_norm], [lhs_z - rhs_z]])

        over_sol = sp.nsolve((esys), [self.q2, self.t], [0, 0], modules=['mpmath'])

        return -q1_sol, over_sol[0], over_sol[1], (self.p01 + Rzq1*self.p12 + Rzq1*Rxq2*p2T).subs([[self.q1, -q1_sol],
                                                                                                   [self.q2, over_sol[0]]])

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
        num_steps = math.ceil(t_elapse / time_step)
        q_steps = np.zeros([self.num_q+1, num_steps])

        for i in range(num_steps):
            # Find current distance along unit vector
            s = norm / (1 + math.e ** (-coeff_s * ((i * time_step) - t0)))

            # Project along unit vector and add initial q
            q_step = s * u + init_q

            q_steps[0, i:i+1] = (i * time_step)
            q_steps[1:3,i:i+1] = q_step

        return q_steps

if __name__ == "__main__":
    gt = GunTurret(100, [-100, 50, 100])
    gt.inverseKin()