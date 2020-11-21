import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from GunTurret import GunTurret
from comms import Comms
import numpy as np
import time

#from RobotRaconteur.Client import *

"""
Supports interface between both simulated and real hardware
Responsible for trickling path matrices at rate specified in path matrices.
"""
class ExamplePub():
    def __init__(self):
        self.name = "examplepub"

        self.gunTurret = GunTurret(0, 0, 40, [-100, 50, 100])

        self.Comms = Comms()
        self.Comms.add_publisher_port('127.0.0.1', '3003', 'gunPath')

    def sendPath(self, path):
        self.Comms.define_and_send(self.name, 'gunPath', path)

    def run(self):
        """
        Check subscriber bus for path matrix. If path is found, clear cameraPath and replace with new path.
        If no path is found,
        """
        while(True):
            q1, q2, toa, f = self.gunTurret.inverseKin()
            q_mat = self.gunTurret.scurvePath(np.array([0, 0]).reshape(2, 1),
                                              np.array([q1, q2]).reshape(2, 1),
                                              6, 1.5, .05)

            self.sendPath(q_mat)
            time.sleep(2)