import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from comms import Comms
import numpy as np
from Engine import Engine
from Target import Target
from CameraTurret import CameraTurret
import time
import queue
from utils.coordination_calculator import xRot, yRot, zRot
#from RobotRaconteur.Client import *

"""
Grabs frames from camera and runs forward kinematics on detected markers
"""
class CameraInterface():
    def __init__(self):
        self.name = "camera"

        # Targets defined here for simulation only
        self.cTurret = CameraTurret([Target(np.array([0, 100, 100]).reshape(3, 1), "t0")])

        self.Comms = Comms()
        self.Comms.add_subscriber_port('127.0.0.1', '3000', 'cState')
        self.Comms.add_publisher_port('127.0.0.1', '3004', 'targetPos')

    def getCState(self):
        try:
            camera_pos = self.Comms.get('cState').payload
            self.cTurret = CameraTurret([Target(np.array([0, 100, 100]).reshape(3, 1), "t0")],
                                        camera_pos[0][0],
                                        camera_pos[1][0])
        except queue.Empty:
            pass

    def grabFrame(self):
        pass

    def publishPos(self, id, target):
        """
        Publishes target coordinates and target id in origin frame to pathing algorithms
        """
        self.Comms.define_and_send(id, 'targetPos', target)

    def run(self):
        prev_frame = time.time()
        # twenty frames per second
        frame_delay = .05

        while(True):
            # If a time delay has passed, grab a frame (in simulation, do nothing)
            if (time.time() - prev_frame >= frame_delay):
                self.grabFrame()
                prev_frame = time.time()

            # Update camera turret coords
            self.getCState()

            # run marker detection on frame (or for simulation, check for targets in range
            targets = self.cTurret.targetsInView()

            #print(targets)

            # if marker detected (or target is within view range for simulation), compute forward kinematics.
            #   for simulation, first derive target in camera frame, then run forward kinematics
            # on target and publish with topic set to marker id.
            R01 = zRot(self.cTurret.q1_given)
            R12 = xRot(self.cTurret.q2_given)
            p_1 = self.cTurret.orig + R01 @ self.cTurret.p12
            p_2 = p_1 + R01 @ R12 @ self.cTurret.pOffset
            t_links = self.cTurret.getTargetLinks(p_2, targets)
            for (t_link, target) in zip(t_links, targets):
                targ_pos = self.cTurret.representTarget(t_link)
                self.publishPos(target.id, targ_pos)
                print(targ_pos)

            time.sleep(.02)