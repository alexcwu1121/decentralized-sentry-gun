import numpy as np
from Entity import Line, Point
from Geometry import Geometry
from Turret import Turret
from utils.coordination_calculator import *
import sympy as sp
import math
import matplotlib.pyplot as plt
import time

class GunTurret(Turret):
    def __init__(self, q1_given, q2_given, v0, p0T,
                 p01=np.array([0, -100, 0]).reshape(3, 1),
                 p12=np.array([3, 0, 50]).reshape(3, 1),
                 pOffset=np.array([-43.5, 0, -11.7]).reshape(3, 1),
                 orig=np.array([100, 50, 0]).reshape(3, 1)):
        super().__init__(q1_given, q2_given, pOffset, p12, orig)
        self.v0 = v0
        self.p01 = p01
        self.num_q = 2

        #  Target
        self.p0T = p0T

        # Trajectory calculations at angle
        self.t = sp.symbols('t')
        self.dv = -(1/2) * 9.81 * sp.cos(self.q2) * self.t**2
        self.dh = -(1/2) * 9.81 * sp.sin(self.q2) * self.t**2 + self.v0 * self.t

        # Derive forms of homogenous transform T and Jacobian J
        self.T, self.J = self.fwdkin()

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
        R12 = yRot(q2)

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
        T12[0:3, 0:3] = yRot_s(self.q2)
        T12[0:3, 3:4] = sp.Matrix(self.p12)

        J = phi(sp.Matrix(yRot_s(self.q1).T), sp.Matrix(self.p12)) * J
        J[0:6, 1:2] = sp.Matrix([1, 0, 0, 0, 0, 0])

        T2T = sp.eye(4)
        # trajectory correction, since initial angle is -15 degrees and not 0
        p2T = sp.Matrix(self.pOffset) + yRot_s(-0.261799) * sp.Matrix([[-self.dh], [0], [self.dv]])
        #p2T = sp.Matrix(self.pOffset) + sp.Matrix([[-self.dh], [0], [self.dv]])
        T2T[0:3, 3:4] = p2T

        # Complex prismatic joint, not along axes
        J = phi(sp.eye(3), sp.Matrix(p2T)) * J
        # Find unit vector of p2T
        p2T_u = p2T/p2T.norm()
        # Insert p2T
        J[0:6, 2:3] = sp.Matrix([0, 0, 0, p2T_u])

        return T01 * T12 * T2T, J

    def inverseKin(self, print_time = False):
        start_time = time.time()

        # Arrange kinematic chain into subproblem 4
        # P0T = P01 + R12 @ P12 + R12 @ R2T @ P2T
        # P0T - P01 = Rz(q1) @ P12 + Rz(q1) @ Ry(q2) @ P2T
        # Rz(q1)T @ (P0T - P01) = P12 + Ry(q2) @ P2T
        # ey * Rz(q1)T @ (P0T - P01) = ey * (P12 + P2T)
        # Where P2T is pOffset + [dh, 0, dv] = [dh + l2; 0; dv + l2']
        eyT = sp.Matrix([[0, 1, 0]])
        Rzq1 = sp.Matrix([[sp.cos(self.q1), -sp.sin(self.q1), 0],
                            [sp.sin(self.q1), sp.cos(self.q1), 0],
                            [0, 0, 1]])

        expr = eyT * Rzq1 * (sp.Matrix(self.p0T) - sp.Matrix(self.p01))

        p2T = sp.Matrix(self.pOffset) + yRot_s(-0.261799) * sp.Matrix([[-self.dh], [0], [self.dv]])
        #p2T = sp.Matrix(self.pOffset) + sp.Matrix([[-self.dh], [0], [self.dv]])
        d = eyT * (sp.Matrix(self.p12) + p2T)

        # Solve kinematic chain for q1. Two solutions.
        q1_sols = sp.solve(expr - d, self.q1)
        print("q1_sols: ", q1_sols)
        q1_sol = q1_sols[0][0]

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

        """
        Ryq2 = sp.Matrix([[1, 0, 0],
                         [0, sp.cos(self.q2), -sp.sin(self.q2)],
                         [0, sp.sin(self.q2), sp.cos(self.q2)]])
        """

        Ryq2 = yRot_s(self.q2)

        rhs = Ryq2 * p2T
        rhs_norm = (rhs[0]**2 + rhs[1]**2 + rhs[2]**2)**.5
        rhs_z = rhs[2]

        esys = sp.Matrix([[lhs_norm - rhs_norm], [lhs_z - rhs_z]])

        over_sol = sp.nsolve((esys), [self.q2, self.t], [np.pi/4, .5], modules=['mpmath'])

        if print_time:
            print(time.time() - start_time)

        return -q1_sol, over_sol[0], over_sol[1], (self.p01 + Rzq1*self.p12 + Rzq1*Ryq2*p2T).subs([[self.q1, -q1_sol],
                                                                                                   [self.q2, over_sol[0]]])

# for testing
if __name__ == "__main__":
    gt = GunTurret(100, [-100, 50, 100])
    gt.inverseKin()