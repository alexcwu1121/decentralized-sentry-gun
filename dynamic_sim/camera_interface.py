import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from comms import Comms
import numpy as np
import cv2
import cv2.aruco as aruco
from Target import Target
from CameraTurret import CameraTurret
import time
import queue
import json
from utils.coordination_calculator import xRot, yRot, zRot
#from RobotRaconteur.Client import *

# Temporary global variables for Aruco dictionary and camera params config locations

"""
Grabs frames from camera and runs forward kinematics on detected markers
"""
class CameraInterface():
    def __init__(self):
        self.name = "camera"

        # Keep track of all targets seen to ensure only one fwdkin is pushed downstream for each
        self.target_log = set()

        # Targets defined here for simulation only
        self.cTurret = CameraTurret([Target(np.array([0, 100, 100]).reshape(3, 1), "t0"),
                                     Target(np.array([0, 100, 150]).reshape(3, 1), "t1"),
                                     Target(np.array([0, 150, 150]).reshape(3, 1), "t2")])

        self.Comms = Comms()
        self.Comms.add_subscriber_port('127.0.0.1', '3000', 'cState')
        self.Comms.add_publisher_port('127.0.0.1', '3004', 'targetPos')

        # twenty frames per second
        self.frame_delay = .05

    def getCState(self):
        try:
            camera_pos = self.Comms.get('cState').payload
            self.cTurret = CameraTurret([Target(np.array([0, 100, 100]).reshape(3, 1), "t0"),
                                         Target(np.array([0, 100, 150]).reshape(3, 1), "t1"),
                                         Target(np.array([0, 150, 150]).reshape(3, 1), "t2")],
                                        camera_pos[0][0],
                                        camera_pos[1][0])
        except queue.Empty:
            pass

    def publishPos(self, id, target):
        """
        Publishes target coordinates and target id in origin frame to pathing algorithms
        """
        self.Comms.define_and_send(id, 'targetPos', target)

    def runS(self):
        prev_frame = time.time()

        while(True):
            # If a time delay has passed, grab a frame (in simulation, do nothing)
            if (time.time() - prev_frame >= self.frame_delay):
                prev_frame = time.time()

            # Update camera turret coords
            self.getCState()

            # run marker detection on frame (or for simulation, check for targets in range)
            targets = self.cTurret.targetsInView()

            # Get new targets and add to log set
            new_targets = {key: targets[key] for key in (targets.keys() - self.target_log)}
            self.target_log = self.target_log | new_targets.keys()

            # if marker detected (or target is within view range for simulation), compute forward kinematics.
            #   for simulation, first derive target in camera frame, then run forward kinematics
            # on target and publish with topic set to marker id.
            R01 = zRot(self.cTurret.q1_given)
            R12 = yRot(self.cTurret.q2_given)
            p_1 = self.cTurret.orig + R01 @ self.cTurret.p12
            p_2 = p_1 + R01 @ R12 @ self.cTurret.pOffset
            t_links = self.cTurret.getTargetLinks(p_2, new_targets.values())

            for (t_link, target) in zip(t_links, new_targets.keys()):
                targ_pos = self.cTurret.representTarget(t_link)
                self.publishPos(target, targ_pos)
                print(targ_pos)

            time.sleep(.02)

    # TODO perform opencv marker detection
    def runR(self):
        # Get marker dictionary set
        marker_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

        # Get default detector parameters
        d_params = aruco.DetectorParameters_create()

        # get camera parameters
        cameraMatrix = np.array([])
        distCoeffs = np.array([])
        with open('cameraData.json', 'r') as cameraData:
            data = json.load(cameraData)
            for camera in data['cameras']:
                if camera['name'] == 'Sahith Mac':
                    parameters = camera['parameters']
                    cameraMatrix = np.array([parameters['lw'], 0, parameters['u0'], 0, parameters['lh'], parameters['v0'], 0, 0, 1]).reshape(3, 3)
                    distCoeffs = np.array([parameters['distortions']])

        # Set up videocap stream
        cap = cv2.VideoCapture(0)

        # time previous frame was captured
        prev_frame = time.time()

        while(True):
            targets = None
            target_ids = None

            # If a time delay has passed, grab a frame (in simulation, do nothing)
            if (time.time() - prev_frame >= self.frame_delay):
                # grab a frame from videocap
                ret, frame = cap.read()

                # Detect markers and corners in frame
                corners, target_ids, rejected = aruco.detectMarkers(frame, marker_dict, parameters=d_params)

                # Estimate marker pose
                _, targets, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)

                prev_frame = time.time()

            # Update camera turret coords
            self.getCState()

            if targets is None:
                continue

            # Get new targets and add to log set
            new_ids = set(target_ids.flatten()) - self.target_log
            self.target_log = self.target_log | set(target_ids.flatten())
            #new_targets = [targets for i in zip(target_ids, targets)]

            # if marker detected, compute forward kinematics.
            for (t_link, id) in zip(targets, target_ids):
                if id not in list(new_ids):
                    continue
                targ_pos = np.array([[-1, 1, 1]]).T * zRot(-np.pi/2) @ xRot(-np.pi/2) @ self.cTurret.representTarget(1000 * t_link.T)
                self.publishPos(id, targ_pos)
                print("Targ {}:".format(id))
                print(targ_pos)
                print()

            time.sleep(.05)

    def run(self, is_sim):
        if is_sim:
            self.runS()
        else:
            self.runR()