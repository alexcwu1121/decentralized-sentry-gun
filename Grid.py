import numpy as np
from Entity import Line

class Grid:
    def __init__(self, ang_len, end_len):
        self.entities = []
        self.ang_len = ang_len
        self.end_len = end_len

        self.initGrid()

    def initGrid(self):
        self.entities = [Line(np.array([0, 0, 0]).reshape(3, 1), np.array([200, 0, 0]).reshape(3, 1), "red", 2),
        # xy angle marker
        Line(np.array([self.ang_len, 0, 0]).reshape(3, 1), np.array([self.ang_len, self.ang_len, 0]).reshape(3, 1), "black", 2),
        Line(np.array([self.ang_len, self.ang_len, 0]).reshape(3, 1), np.array([0, self.ang_len, 0]).reshape(3, 1), "black", 2),
        Line(np.array([200, 0, 0]).reshape(3, 1), np.array([200, self.end_len, 0]).reshape(3, 1), "red", 2),
        Line(np.array([200, 0, 0]).reshape(3, 1), np.array([200, 0, self.end_len]).reshape(3, 1), "red", 2),

        Line(np.array([0, 0, 0]).reshape(3, 1), np.array([0, 200, 0]).reshape(3, 1), "green", 2),
        # yz angle marker
        Line(np.array([0, self.ang_len, 0]).reshape(3, 1), np.array([0, self.ang_len, self.ang_len]).reshape(3, 1), "black", 2),
        Line(np.array([0, self.ang_len, self.ang_len]).reshape(3, 1), np.array([0, 0, self.ang_len]).reshape(3, 1), "black", 2),
        Line(np.array([0, 200, 0]).reshape(3, 1), np.array([self.end_len, 200, 0]).reshape(3, 1), "green", 2),
        Line(np.array([0, 200, 0]).reshape(3, 1), np.array([0, 200, self.end_len]).reshape(3, 1), "green", 2),

        Line(np.array([0, 0, 0]).reshape(3, 1), np.array([0, 0, 200]).reshape(3, 1), "blue", 2),
        # zx angle marker
        Line(np.array([0, 0, self.ang_len]).reshape(3, 1), np.array([self.ang_len, 0, self.ang_len]).reshape(3, 1), "black", 2),
        Line(np.array([self.ang_len, 0, self.ang_len]).reshape(3, 1), np.array([self.ang_len, 0, 0]).reshape(3, 1), "black", 2),
        Line(np.array([0, 0, 200]).reshape(3, 1), np.array([0, self.end_len, 200]).reshape(3, 1), "blue", 2),
        Line(np.array([0, 0, 200]).reshape(3, 1), np.array([self.end_len, 0, 200]).reshape(3, 1), "blue", 2)]

    def getEntities(self):
        return self.entities
