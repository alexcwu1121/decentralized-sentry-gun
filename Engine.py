import numpy as np
from tkinter import Tk, Canvas
import time
from utils import coordination_calculator as cc
from Grid import Grid

class Engine:
    def __init__(self, c_trans, c_rot):
        self.canvas_width = 640
        self.canvas_height = 480
        self.c_trans = c_trans
        self.c_rot = c_rot

        # Initialize camera rotation matrix
        self.c_rotmat = cc.Rodriguez(self.c_rot[0][0], self.c_rot[1][0], self.c_rot[2][0])

        self.initTk()

        self.entities = []

    def initTk(self):
        self.master = Tk()
        self.canvas = Canvas(self.master,
                   width=self.canvas_width,
                   height=self.canvas_height)
        self.canvas.pack()

    def addEntities(self, entities):
        self.entities.extend(entities)

    def update(self):
        for entity in self.entities:
            entity.draw(self.canvas, self.c_rotmat, self.canvas_width, self.canvas_height)
        self.master.update()

if __name__ == "__main__":
    e = Engine(np.array([150, 150, 150]).reshape(3, 1),
               np.array([np.pi/2-.5, np.pi/4, .3]).reshape(3, 1))

    e.addEntities(Grid(20, 20).getEntities())

    while(True):
        e.update()
        time.sleep(1)