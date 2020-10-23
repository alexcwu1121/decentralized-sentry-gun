import numpy as np
from Entity import Line, Point, DottedLine
from Geometry import Geometry
from utils.coordination_calculator import xRot, yRot, zRot


class CameraTurret(Geometry):
    def __init__(self, targets = None, q1=0, q2=0, pOffset=np.array([0, 50, 0]).reshape(3, 1),
                 p12=np.array([0, 0, 75]).reshape(3, 1),
                 orig=np.array([100, 50, 0]).reshape(3, 1)):
        super().__init__()
        self.q1 = q1
        self.q2 = q2
        self.pOffset = pOffset
        self.p12 = p12
        self.orig = orig
        self.targets = targets

    def getEntities(self):
        R01 = zRot(self.q1)
        R12 = xRot(self.q2)

        p_1 = self.orig + R01 @ self.p12
        p_2 = p_1 + R01 @ R12 @ self.pOffset

        target_p = self.orig if self.targets is None else self.targets[0].pos

        return [Line(np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), p_2, "black", 2),
                Line(np.array([0, p_2[1][0], 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),
                Line(np.array([p_2[0][0], 0, 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),

                Line(self.orig, p_1, "orange", 8),
                Line(p_1, p_2, "orange", 8),
                DottedLine(self.orig, self.representTarget(target_p), "red", 2, 10),
                Point(self.orig, 'black', 10),
                Point(p_1, 'black', 10),
                Point(p_2, 'black', 10),
                ]

    def representTarget(self, distance=np.array([0, 0, 0]).reshape(3, 1)):
        # Define POE parameters
        # h1 = [0, 0, 1]T or ez
        # h2 = [1, 0, 0]T or ex
        # p01 = [0, 0, 0]T
        # p12 = self.p12, default case [0, 0, 75]T
        # p2T = self.pOffset + {target vector}
        p01 = np.array([0, 0, 0]).reshape(3, 1)
        p2T = self.pOffset + distance

        # Define rotation matrices
        # e^(h1xq1) = Rz(q1)
        # e^(h2xq2) = Rx(q2)
        R01 = zRot(self.q1)
        R12 = xRot(self.q2)

        # Define HTMS
        # T01 = |R01    p01|
        #       |0      1  |
        T01 = np.identity(4)
        T01[0:3, 0:3] = R01
        T01[0:3, 3:4] = p01

        # T12 = |R12    p12|
        #       |0      1  |
        T12 = np.identity(4)
        T12[0:3, 0:3] = R12
        T12[0:3, 3:4] = self.p12

        # T2T = |I      p2T|
        #       |0      1  |
        T2T = np.identity(4)
        T2T[0:3, 3:4] = p2T

        target = T01 @ T12 @ T2T

        return target[0:3, 3:4]

