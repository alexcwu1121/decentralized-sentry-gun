import numpy as np
from Entity import Point
from Geometry import Geometry

class Target(Geometry):
    def __init__(self, pos):
        super().__init__()

        self.pos = pos

    def getEntities(self):
        return [Point(self.pos, 'red', 20)]
