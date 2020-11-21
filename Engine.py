import numpy as np
from tkinter import Tk, Canvas
import time
from utils import coordination_calculator as cc
from Grid import Grid
from Target import Target
from CameraTurret import CameraTurret
from GunTurret import GunTurret

class Engine:
    def __init__(self, c_trans, c_rot):
        # Camera frame and orientation parameters
        self.canvas_width = 640
        self.canvas_height = 480
        self.c_trans = c_trans
        self.c_rot = c_rot

        # Initialize camera rotation matrix
        self.c_rotmat = cc.Rodriguez(self.c_rot[0][0], self.c_rot[1][0], self.c_rot[2][0])

        # Initialize tk context and canvas
        self.initTk()

        # List of objects to be drawn
        self.geometries = []

    def initTk(self):
        self.master = Tk()
        self.canvas = Canvas(self.master,
                   width=self.canvas_width,
                   height=self.canvas_height)
        self.canvas.pack()

    def addGeometry(self, objects):
        self.geometries.extend(objects)

    def update(self):
        for geometry in self.geometries:
            for entity in geometry.getEntities():
                entity.draw(self.canvas, self.c_rotmat, self.canvas_width, self.canvas_height)
        self.master.update()


if __name__ == "__main__":
    e = Engine(np.array([150, 150, 150]).reshape(3, 1),
               np.array([np.pi/2-.4, np.pi/4 + .2, .3]).reshape(3, 1))

    e.addGeometry([Grid(20, 20)])
    e.addGeometry([Target(np.array([0, 100, 100]).reshape(3, 1))])
    e.addGeometry([CameraTurret([Target(np.array([0, 100, 100]).reshape(3, 1))])])
    e.addGeometry([GunTurret(40, [-100, 50, 100])])

    current_configuration = "initial_configuration"
    count = 0
    #time = 0
    period = 0.01
    stop_time = 999
    moving = False
    while True:
        e.update()
        # if not moving:
        #     coordinate = "receive from camera turret"
        #     configuration_matrix = "calculate path with current configuration and the coordinate"
        #     run_time = "should return together with the matrix"
        #     period = float(run_time) / (len(configuration_matrix) - 1)
        #     stop_time = time.clock() + run_time
        # else:
        #     if time.clock() <= stop_time:
        #         current_configuration = configuration_matrix[count]
        #         count += 1
        #     else:
        #         move = False
        time.sleep(1)


