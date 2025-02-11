import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from CameraTurret import CameraTurret
from comms import Comms
import numpy as np
import time

#from RobotRaconteur.Client import *

"""
Example camera motion publisher to test correctness of camera path
"""
class ExampleCameraPub():
    def __init__(self):
        self.name = "examplecamera"

        self.cameraTurret = CameraTurret()

        self.Comms = Comms()
        self.Comms.add_publisher_port('127.0.0.1', '3002', 'cameraPath')

    def sendPath(self, path):
        self.Comms.define_and_send(self.name, 'cameraPath', path)

    def run(self, is_sim):
        """
        Check subscriber bus for path matrix. If path is found, clear cameraPath and replace with new path.
        If no path is found,
        """  
        # Testing required to find q1_max and q2_max
        while(True):
            q_mat = self.cameraTurret.sweepPath(5, q1_range=(-np.pi/4,np.pi/4), q2_range=(-np.pi/4,np.pi/4))

            self.sendPath(q_mat)
            time.sleep(5)