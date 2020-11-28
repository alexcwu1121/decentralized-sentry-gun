import numpy as np
from GunTurret import GunTurret
from comms import Comms
import time
import queue


class GunMotion():
    def __init__(self):
        self.name = "gunmotion"
        self.state = 0
        self.configuration = np.array([[0, 0]]).T
        self.target = np.array([[0, 0]])
        self.pathMatrix = None
        self.Comms = Comms()
        self.Comms.add_publisher_port('127.0.0.1', '3003', 'gunPath')

    def receive(self):
        try:
            self.configuration = self.Comms.get('gState').payload
        except queue.Empty:
            pass
        try:
            self.target = self.Comms.get('targetPos').payload
        except queue.Empty:
            pass

    def publish(self):
        self.Comms.define_and_send(self.name, 'gunPath', self.pathMatix)

    def run(self):
        while True:
            self.receive()
            if self.state == 1:
                q1, q2, toa, f = GunTurret.inverseKin()
                self.pathMatix = GunTurret.scurvePath(np.array([0, 0]).reshape(2, 1),
                                                  np.array([q1, q2]).reshape(2, 1),
                                                  6, 1.5, .05)
                self.publish()
            time.sleep(.02)