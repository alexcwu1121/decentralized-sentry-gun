import numpy as np
from Entity import Line, Point, DottedLine
from Geometry import Geometry
from utils.coordination_calculator import xRot, yRot, zRot
import sympy as sp
from math import sin, cos

class GunTurret(Geometry):
    def __init__(self, v0, p0T,
                 p01=np.array([0, -100, 0]).reshape(3, 1),
                 p12=np.array([0, 0, 75]).reshape(3, 1),
                 pOffset=np.array([0, 50, 0]).reshape(3, 1),
                 orig=np.array([100, 50, 0]).reshape(3, 1)):
        super().__init__()
        self.v0 = v0
        self.p01 = p01
        self.p12 = p12
        self.pOffset = pOffset
        self.orig = orig

        #  Target
        self.p0T = p0T

        # Trajectory calculations at angle
        self.q1, self.q2, self.t = sp.symbols('q1 q2 t')
        self.dv = -(1/2) * 9.81 * sp.sin(self.q2) * self.t**2
        self.dh = -(1/2) * 9.81 * sp.cos(self.q2) * self.t**2 + self.v0 * self.t

    def getEntities(self):
        q1, q2, toa, f = self.inverseKin()

        R01 = zRot(q1)
        R12 = xRot(q2)

        p_1 = self.orig + self.p01 + R01 @ self.p12
        p_2 = p_1 + R01 @ R12 @ self.pOffset

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
        interval = toa/steps
        entities = []
        for i in range(steps + 1):
            t = i * interval
            p = f.subs(self.t, t)
            np_p = self.orig + np.array(p).astype(np.float64)
            entities.append(Point(np_p, 'black', 2))
        return entities

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
        #print(lhs_norm)
        lhs_z = lhs_val[2]

        Rxq2 = sp.Matrix([[1, 0, 0],
                         [0, sp.cos(self.q2), -sp.sin(self.q2)],
                         [0, sp.sin(self.q2), sp.cos(self.q2)]])

        rhs = Rxq2 * p2T
        rhs_norm = (rhs[0]**2 + rhs[1]**2 + rhs[2]**2)**.5
        #print(rhs_norm)
        rhs_z = rhs[2]

        esys = sp.Matrix([[lhs_norm - rhs_norm], [lhs_z - rhs_z]])

        over_sol = sp.nsolve((esys), [self.q2, self.t], [0, 0], modules=['mpmath'])
        #print(over_sol)

        return -q1_sol, over_sol[0], over_sol[1], (self.p01 + Rzq1*self.p12 + Rzq1*Rxq2*p2T).subs([[self.q1, -q1_sol],
                                                                                                   [self.q2, over_sol[0]]])

if __name__ == "__main__":
    gt = GunTurret(100, [-100, 50, 100])
    gt.inverseKin()