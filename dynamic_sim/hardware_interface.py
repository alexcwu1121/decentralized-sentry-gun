import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from comms import Comms
import numpy as np
from Engine import Engine
from Grid import Grid
from Target import Target
from CameraTurret import CameraTurret
from GunTurret import GunTurret
import serial
import threading
import time
import queue
#from RobotRaconteur.Client import *

"""
Supports interface between both simulated and real hardware
Responsible for trickling path matrices at rate specified in path matrices.
"""
class HardwareInterface():
    def __init__(self):
        self.name = "hardware"
        self.gun_config = np.array([[0, 0]]).T
        self.camera_config = np.array([[0, 0]]).T
        self.gun_ready = 0

        self.gunReady = True
        self.gunTurretReady = False

        # Establish serial port and baud rate
        # self.ard = serial.Serial('COM3', 9600)
        # if (self.ard.isOpen() == False):
        #     self.ard.open()

        # When a target has been detected and
        self.cameraPath = None
        self.gunPath = None

        # Simulation graphics engine
        self.sim_out = Engine(np.array([150, 150, 150]).reshape(3, 1),
                   np.array([np.pi / 2 - .4, np.pi / 4 + .2, .3]).reshape(3, 1))

        self.Comms = Comms()
        self.Comms.add_publisher_port('127.0.0.1', '3000', 'cState')
        self.Comms.add_publisher_port('127.0.0.1', '3001', 'gState')
        self.Comms.add_subscriber_port('127.0.0.1', '3002', 'cameraPath')
        self.Comms.add_subscriber_port('127.0.0.1', '3003', 'gunPath')

    def readStateR(self):
        line = []
        while True:
            for c in self.ard.read():
                if chr(c) == ':':
                    # parse return from arduino
                    #print("".join(line))

                    # Run comma delimited parse on [2:]
                    parsed = "".join(line[2:]).split(',')
                    if (line[0] == 'c'):
                        self.camera_config[0][0] = float(parsed[0])
                        self.camera_config[1][0] = float(parsed[1])
                    elif (line[0] == 'g'):
                        self.gun_config[0][0] = float(parsed[0])
                        self.gun_config[1][0] = float(parsed[1])
                    elif (line[0] == 's'):
                        self.gun_ready = float(parsed[0])

                    """
                    print(camera_config)
                    print(gun_config)
                    print(gun_ready)
                    """

                    line = []
                    break
                else:
                    line.append(chr(c))

    def readStateS(self):
        """
        Reads servo configurations and updates internal state (simulation)
        """
        # Do nothing, since ideal situation has interface state always equal to actual hardware state
        self.camera_config = self.camera_config
        self.gun_config = self.gun_config

    def writeCamR(self, ctarg):
        """
        Writes a servo configuration (hardware)
        """
        self.ard.write("c,{},{}:".format(ctarg[0][0], ctarg[1][0]).encode())

    def writeCamS(self, ctarg):
        """
        Writes a servo configuration (simulation)
        """
        # Updates engine with new configurations and draws
        self.sim_out.clearGeometries()

        self.camera_config = ctarg

        # Grid and target dimensions are hardcoded
        self.drawSim(ctarg, self.gun_config)

        #self.sim_out.update()

    def writeGunR(self, gtarg):
        self.ard.write("g,{},{}:".format(gtarg[0][0], gtarg[1][0]).encode())

    def writeGunS(self, gtarg):
        """
        Writes a servo configuration (simulation)
        """
        # Updates engine with new configurations and draws
        self.sim_out.clearGeometries()

        self.gun_config = gtarg

        # Grid and target dimensions are hardcoded
        self.drawSim(self.camera_config, gtarg)

        # self.sim_out.update()

    def writeShootR(self):
        self.ard.write("s:".encode())

    def publishState(self):
        """
        Publishes current configuration states of gun turret and camera turret to both publisher ports
        """
        self.Comms.define_and_send(self.name, 'cState', self.camera_config)
        self.Comms.define_and_send(self.name, 'gState', self.gun_config)

    def receivePath(self):
        """
        Pulls path matrices from subscriber bus and replace current path matrices
        """
        try:
            self.cameraPath = self.Comms.get('cameraPath').payload
        except queue.Empty:
            pass

        try:
            self.gunPath = self.Comms.get('gunPath').payload
        except queue.Empty:
            pass

    def drawSim(self, cameraPos, gunPos):
        self.sim_out.addGeometry([Grid(20, 20)])
        self.sim_out.addGeometry([Target(np.array([0, 100, 100]).reshape(3, 1), "t0")])
        self.sim_out.addGeometry([CameraTurret([Target(np.array([0, 100, 100]).reshape(3, 1), "t0")],
                                    cameraPos[0, 0],
                                    cameraPos[1, 0]
                                    )])
        self.sim_out.addGeometry([GunTurret(gunPos[0, 0],
                                 gunPos[1, 0],
                                 40, [-100, 50, 100])])


    # def fireTheShot(self):
    #     # TODO: Send a fire command to the gun to Arduino and start reloading
    #     self.gunReady = False

    # def checkTurretReady(self):
    #     """
    #     Check if the turret is aiming at the target
    #     """
    #     error = 0.02
    #     if self.gunPath[-1][0] * (1 - error) < self.gun_config[0] < self.gunPath[-1][0] * (1 + error) and \
    #        self.gunPath[-1][1] * (1 - error) < self.gun_config[1] < self.gunPath[-1][1] * (1 + error):
    #         self.gunReady = True
    #     else:
    #         self.gunReady = False

    def runS(self):
        """
        Check subscriber bus for path matrix. If path is found, clear cameraPath and replace with new path.
        If no path is found,
        """
        prev_gun_write = time.time()
        gun_write_delay = 0

        prev_camera_write = time.time()
        camera_write_delay = 0

        self.drawSim(self.camera_config, self.gun_config)

        while(True):
            # Update configuration states and publish them
            self.readStateS()
            self.publishState()
            #self.checkTurretReady()
            # Pull path matrix from queue and replace existing matrices
            self.receivePath()

            # If path steps exist and sufficient time has elapsed since the last write, write again
            if self.gunPath is not None and self.gunPath.shape[1] != 0 and\
               time.time() - prev_gun_write >= gun_write_delay:

                # Write config
                self.writeGunS(self.gunPath[1:3, 0:1])
                prev_gun_write = time.time()

                # Set new delay
                if self.gunPath.shape[1] > 1:
                    # subtract next timestamp from current timestamp for delay
                    gun_write_delay = self.gunPath[0][1] - self.gunPath[0][0]
                else:
                    gun_write_delay = 0

                # Pop current config
                self.gunPath = self.gunPath[0:3, 1:]

            if self.cameraPath is not None and self.cameraPath.shape[1] != 0 and\
               time.time() - prev_camera_write >= camera_write_delay:

                # Write config
                self.writeCamS(self.cameraPath[1:3, 0:1])
                prev_camera_write = time.time()

                # Set new delay
                if self.cameraPath.shape[1] > 1:
                    # subtract next timestamp from current timestamp for delay
                    camera_write_delay = self.cameraPath[0][1] - self.cameraPath[0][0]
                else:
                    camera_write_delay = 0

                # Pop current config
                self.cameraPath = self.cameraPath[0:3, 1:]
            # if self.gunReady and self.gunTurretReady:
            #     self.fireTheShot()
            # Display simulation.
            self.sim_out.update()
            # Hardware interface updates 50 times a second
            time.sleep(.02)

    def runR(self):
        """
        Check subscriber bus for path matrix. If path is found, clear cameraPath and replace with new path.
        If no path is found,
        """
        prev_gun_write = time.time()
        gun_write_delay = 0

        prev_camera_write = time.time()
        camera_write_delay = 0

        t = threading.Thread(target=self.readStateR(), args=())
        t.start()

        while(True):
            # Update configuration states and publish them
            self.readStateR()
            self.publishState()

            # Pull path matrix from queue and replace existing matrices
            self.receivePath()

            # If path steps exist and sufficient time has elapsed since the last write, write again
            if  (self.gunPath is not None) and\
                    (self.gunPath.shape[1] != 0) and\
                    (time.time() - prev_gun_write >= gun_write_delay):

                # Write config
                self.writeGunR(self.gunPath[1:3, 0:1])
                prev_gun_write = time.time()

                # Set new delay
                if(self.gunPath.shape[1] > 1):
                    # subtract next timestamp from current timestamp for delay
                    gun_write_delay = self.gunPath[0][1] - self.gunPath[0][0]
                else:
                    gun_write_delay = 0

                # Pop current config
                self.gunPath = self.gunPath[0:3, 1:]

            if (self.cameraPath is not None) and\
                    (self.cameraPath.shape[1] != 0) and\
                    (time.time() - prev_camera_write >= camera_write_delay):

                # Write config
                self.writeCamR(self.cameraPath[1:3, 0:1])
                prev_camera_write = time.time()

                # Set new delay
                if(self.cameraPath.shape[1] > 1):
                    # subtract next timestamp from current timestamp for delay
                    camera_write_delay = self.cameraPath[0][1] - self.cameraPath[0][0]
                else:
                    camera_write_delay = 0

                # Pop current config
                self.cameraPath = self.cameraPath[0:3, 1:]

            # Hardware interface updates 50 times a second
            time.sleep(.02)

    def run(self, is_sim):
        if is_sim:
            self.runS()
        else:
            self.runR()
