import numpy as np
from Entity import Point, Line
from Geometry import Geometry

class Target(Geometry):
    def __init__(self, pos):
        super().__init__()

        self.pos = pos

    def getEntities(self):
        return [Line(np.array([0, self.pos[1][0], self.pos[2][0]]).reshape(3, 1), self.pos, "red", 2),
                Line(np.array([0, self.pos[1][0], self.pos[2][0]]).reshape(3, 1),
                     np.array([0, self.pos[1][0], 0]).reshape(3, 1), "red", 2),
                Line(np.array([0, self.pos[1][0], self.pos[2][0]]).reshape(3, 1),
                     np.array([0, 0, self.pos[2][0]]).reshape(3, 1), "red", 2),
            Point(self.pos, 'red', 20)]
