import numpy as np
from GunTurret import GunTurret
from comms import Comms
import time
import queue


class GunMotion():
    def __init__(self):
        self.name = "gunmotion"
        self.configuration = np.array([[0, 0]]).T
        self.target = None
        self.pathMatrix = None
        self.Comms = Comms()
        self.Comms.add_publisher_port('127.0.0.1', '3003', 'gunPath')

    def receive(self):
        """
        Reads gun turret configurations and the target position
        """
        try:
            self.configuration = self.Comms.get('gState').payload
        except queue.Empty:
            pass
        try:
            self.target = self.Comms.get('targetPos').payload
        except queue.Empty:
            pass

    def publish(self):
        """
        Publishes the gun path
        """
        self.Comms.define_and_send(self.name, 'gunPath', self.pathMatix)

    def run(self):
        """
        Checks the configuration of gun turret and the target, if a target is recieved, calculate the path matix and
        delete the target to wait for the next one
        """
        while True:
            self.receive()
            if self.target != None:
                q1, q2, toa, f = GunTurret.inverseKin()
                self.pathMatix = GunTurret.scurvePath(np.array([0, 0]).reshape(2, 1),
                                                      np.array([q1, q2]).reshape(2, 1),
                                                      6, 1.5, .05)
                self.target = None
                self.publish()
            time.sleep(.02)
