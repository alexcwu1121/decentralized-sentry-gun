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
        self.p01 = orig + p01
        self.p12 = p12
        self.pOffset = pOffset

        #  Target
        self.p0T = p0T

        # Trajectory calculations at angle
        self.q1, self.q2, self.t = sp.symbols('q1 q2 t')
        self.dv = -(1/2) * 9.81 * sp.sin(self.q2) * self.t**2
        self.dh = -(1/2) * 9.81 * sp.cos(self.q2) * self.t**2 + self.v0 * self.t

    def getEntities(self):
        return 0

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

        # Substitute q1 into kinematic chain and solve for q2, again using subproblem 4
        # POT - P01 - R12 @ P12 = R12 @ R2T @ P2T
        # R12T @ (POT - P01 - R12 @ P12) = R2T @ P2T
        # ezT * R12T @ (POT - P01 - R12 @ P12) = ezT * R2T @ P2T

        lhs = Rzq1.T * (sp.Matrix(self.p0T) - sp.Matrix(self.p01) - Rzq1 * sp.Matrix(self.p12))
        lhs_val = lhs.subs(self.q1, q1_sol)
        print(lhs_val)

        Rxq2 = sp.Matrix([[1, 0, 0],
                         [0, sp.cos(self.q2), -sp.sin(self.q2)],
                         [0, sp.sin(self.q2), sp.cos(self.q2)]])

        rhs = Rxq2 * p2T
        rhs_val = rhs.subs(self.q1, q1_sol)
        print(rhs_val)

        #over_sol = sp.nonlinsolve(lhs_val - rhs_val, [self.q2, self.t])
        over_sol = sp.nsolve((lhs_val - rhs_val)[1:4,:], [self.q2, self.t], [0, 0], modules = ['mpmath'])
        print(over_sol)

        print(rhs_val.subs([(self.q2, over_sol[0]), (self.t, over_sol[1])]))

        return 0

if __name__ == "__main__":
    gt = GunTurret(300, [0, 100, 100])
    gt.inverseKin()