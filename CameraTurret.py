import numpy as np
from Entity import Line, Point
from Geometry import Geometry
from utils.coordination_calculator import xRot, yRot, zRot

class CameraTurret(Geometry):
    def __init__(self, q1=0, q2=0, pOffset=np.array([0, 50, 0]).reshape(3,1),
                 p01=np.array([0, 0, 75]).reshape(3,1),
                 orig=np.array([100, 50, 0]).reshape(3,1)):
        super().__init__()
        self.q1 = q1
        self.q2 = q2
        self.pOffset = pOffset
        self.p01 = p01
        self.orig = orig

    def getEntities(self):
        R01 = zRot(self.q1)
        R12 = xRot(self.q2)

        p_1 = self.orig + R01 @ self.p01
        p_2 = p_1 + R01 @ R12 @ self.pOffset

        return [Line(np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), p_2, "black", 2),
                Line(np.array([0, p_2[1][0], 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), "black", 2),
                Line(np.array([p_2[0][0], 0, 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), "black", 2),

                Line(self.orig, p_1, "orange", 8),
                Line(p_1, p_2, "orange", 8),
                Point(self.orig, 'black', 10),
                Point(p_1, 'black', 10),
                Point(p_2, 'black', 10)
        ]

