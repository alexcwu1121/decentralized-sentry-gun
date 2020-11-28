import numpy as np
from Entity import Point, Line
from Geometry import Geometry

class Target(Geometry):
    def __init__(self, pos, id):
        super().__init__()
        self.id = id
        self.pos = pos

    def getIds(self):
        return self.id

    def getEntities(self):
        return [Line(np.array([0, self.pos[1][0], self.pos[2][0]]).reshape(3, 1), self.pos, "red", 2),
                Line(np.array([0, self.pos[1][0], self.pos[2][0]]).reshape(3, 1),
                     np.array([0, self.pos[1][0], 0]).reshape(3, 1), "red", 2),
                Line(np.array([0, self.pos[1][0], self.pos[2][0]]).reshape(3, 1),
                     np.array([0, 0, self.pos[2][0]]).reshape(3, 1), "red", 2),
            Point(self.pos, 'red', 20)]
