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

        self.gTurret = GunTurret(0, 0, 70, [-100, 50, 100])

        self.Comms = Comms()
        self.Comms.add_subscriber_port('127.0.0.1', '3001', 'gState')
        self.Comms.add_subscriber_port('127.0.0.1', '3004', 'targetPos')
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
            self.gTurret.setP0T(self.Comms.get('targetPos').payload)
            self.target = True
        except queue.Empty:
            pass

    def publish(self):
        """
        Publishes the gun path
        """
        self.Comms.define_and_send(self.name, 'gunPath', self.pathMatrix)

    def run(self, is_sim):
        """
        Checks the configuration of gun turret and the target, if a target is received, calculate the path matrix and
        delete the target to wait for the next one
        """
        while True:
            self.receive()
            if self.target:
                q1, q2, toa, f = self.gTurret.inverseKin(print_time=True)
                self.pathMatrix = self.gTurret.scurvePath(self.configuration,
                                                         np.array([q1, q2]).reshape(2, 1),
                                                         10, 1.5, .05)

                self.target = False
                self.publish()
            time.sleep(.02)
