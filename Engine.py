import numpy as np
from tkinter import Tk, Canvas
import time
import sys
import cv2
from utils import coordination_calculator as cc
from abc import ABC, abstractmethod

class Entity(ABC):
    def __init__(self, color, thickness):
        self.color = color
        self.thickness = thickness

    def draw(self, canvas):
        pass

class Dot(Entity):
    def __init__(self, p, color, diameter):
        super().__init__(color, diameter)
        self.p = p

    def draw(self, canvas):
        x0 = self.p[0] - self.thickness/2
        y0 = self.p[1] - self.thickness/2
        x1 = self.p[0] + self.thickness/2
        y1 = self.p[1] + self.thickness/2
        canvas.create_oval(x0, y0, x1, y1)

class Line(Entity):
    def __init__(self, p1, p2, color, thickness):
        super().__init__(color, thickness)
        self.p1 = p1
        self.p2 = p2

    def draw(self, canvas):
        print(">>>>>>>>")
        print(self.p1)
        print(self.p2)
        canvas.create_line(self.p1[0][0], self.p1[1][0],
                     self.p2[0][0], self.p2[1][0],
                     fill=self.color,
                     width=self.thickness)

class Engine:
    def __init__(self, c_trans, c_rot, ang_len=20, end_len=20):
        self.canvas_width = 640
        self.canvas_height = 480
        self.c_trans = c_trans
        self.c_rot = c_rot
        self.ang_len = ang_len
        self.end_len = end_len

        self.init_tk()

        self.entities = []

    def init_tk(self):
        self.master = Tk()
        self.canvas = Canvas(self.master,
                   width=self.canvas_width,
                   height=self.canvas_height)
        self.canvas.pack()

    # rotate a point from world space to camera space and cut out z coordinate
    # Shift every point to center screen
    def projection(self, p):
        #c_rotmat,_ = cv2.Rodriguez(self.c_rot)
        c_rotmat = cc.Rodriguez(self.c_rot[0][0], self.c_rot[1][0], self.c_rot[2][0])
        print(c_rotmat)
        p_c = np.matmul(c_rotmat, p)
        p_cimage = np.array([p_c[0][0] + self.canvas_width/2, p_c[1][0] + self.canvas_height/2]).reshape(2,1)
        return p_cimage

    # Todo: make grid its own entity
    def init_grid(self):
        self.addLine(np.array([0, 0, 0]).reshape(3, 1), np.array([200, 0, 0]).reshape(3, 1), "red", 2)
        # xy angle marker
        self.addLine(np.array([self.ang_len, 0, 0]).reshape(3, 1), np.array([self.ang_len, self.ang_len, 0]).reshape(3, 1), "black", 2)
        self.addLine(np.array([self.ang_len, self.ang_len, 0]).reshape(3, 1), np.array([0, self.ang_len, 0]).reshape(3, 1), "black", 2)
        self.addLine(np.array([200, 0, 0]).reshape(3, 1), np.array([200, self.end_len, 0]).reshape(3, 1), "red", 2)
        self.addLine(np.array([200, 0, 0]).reshape(3, 1), np.array([200, 0, self.end_len]).reshape(3, 1), "red", 2)

        self.addLine(np.array([0, 0, 0]).reshape(3, 1), np.array([0, 200, 0]).reshape(3, 1), "green", 2)
        # yz angle marker
        self.addLine(np.array([0, self.ang_len, 0]).reshape(3, 1), np.array([0, self.ang_len, self.ang_len]).reshape(3, 1), "black", 2)
        self.addLine(np.array([0, self.ang_len, self.ang_len]).reshape(3, 1), np.array([0, 0, self.ang_len]).reshape(3, 1), "black", 2)
        self.addLine(np.array([0, 200, 0]).reshape(3, 1), np.array([self.end_len, 200, 0]).reshape(3, 1), "green", 2)
        self.addLine(np.array([0, 200, 0]).reshape(3, 1), np.array([0, 200, self.end_len]).reshape(3, 1), "green", 2)

        self.addLine(np.array([0, 0, 0]).reshape(3, 1), np.array([0, 0, 200]).reshape(3, 1), "blue", 2)
        # zx angle marker
        self.addLine(np.array([0, 0, self.ang_len]).reshape(3, 1), np.array([self.ang_len, 0, self.ang_len]).reshape(3, 1), "black", 2)
        self.addLine(np.array([self.ang_len, 0, self.ang_len]).reshape(3, 1), np.array([self.ang_len, 0, 0]).reshape(3, 1), "black", 2)
        self.addLine(np.array([0, 0, 200]).reshape(3, 1), np.array([0, self.end_len, 200]).reshape(3, 1), "blue", 2)
        self.addLine(np.array([0, 0, 200]).reshape(3, 1), np.array([self.end_len, 0, 200]).reshape(3, 1), "blue", 2)

    def addLine(self, p1_3D, p2_3D, color, thickness):
        p1_cimage = self.projection(p1_3D)
        p2_cimage = self.projection(p2_3D)
        self.entities.append(Line(p1_cimage, p2_cimage, color, thickness))

    #def addPoint(self):

    def update(self):
        for entity in self.entities:
            entity.draw(self.canvas)
        self.master.update()

if __name__ == "__main__":
    e = Engine(np.array([150, 150, 150]).reshape(3, 1),
               np.array([np.pi/2-.5, np.pi/4, .3]).reshape(3, 1))
    e.init_grid()
    e.update()
    time.sleep(10)