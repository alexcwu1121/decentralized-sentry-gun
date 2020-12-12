import numpy as np
from GunTurret import GunTurret
from comms import Comms
import time
import queue


class GunMotion():
    def __init__(self):
        self.name = "gunmotion"

        # Current configuration of gun turret as reported by hardware
        self.actual_configuration = np.array([[0, 0]]).T
        self.dest_configuration = np.array([[0, 0]]).T

        # Configuration comparison tolerance (rad)
        self.tol = .025

        # First target indicator to skip delay
        self.first_targ = True

        # Time to wait for gun to fire
        self.delay = 2

        # queue of P0Ts to process
        self.targets = []

        self.gTurret = GunTurret(0, 0, 500, [-100, 50, 100])

        self.Comms = Comms()
        self.Comms.add_subscriber_port('127.0.0.1', '3001', 'gState')
        self.Comms.add_subscriber_port('127.0.0.1', '3004', 'targetPos')
        self.Comms.add_publisher_port('127.0.0.1', '3003', 'gunPath')

    def receive(self):
        """
        Reads gun turret configurations and the target position
        """
        try:
            self.actual_configuration = self.Comms.get('gState').payload
        except queue.Empty:
            pass
        try:
            self.targets.append(self.Comms.get('targetPos').payload)
        except queue.Empty:
            pass

    def publish(self, pathMatrix):
        """
        Publishes the gun path
        """
        self.Comms.define_and_send(self.name, 'gunPath', pathMatrix)

    def compareTol(self):
        # If there is a destination, see if configs match
        #for q, q_prime in zip(self.actual_configuration, self.dest_configuration):
        for i in range(self.actual_configuration.shape[0]):
            if (self.actual_configuration[i][0] > self.dest_configuration[i][0] + self.tol or
                    self.actual_configuration[i][0] < self.dest_configuration[i][0] - self.tol):
                return False

        # Delay for as long as the gun takes to wind up and shoot
        # For real demo, about 6 seconds
        if self.first_targ:
            self.first_targ = False
        else:
            time.sleep(self.delay)

        # Allow another path matrix to be sent
        return True

    def run(self, is_sim):
        """
        Checks the configuration of gun turret and the target, if a target is received, calculate the path matrix and
        delete the target to wait for the next one
        """
        while True:
            self.receive()
            """
            if self.prev_path_time is not None and time.time() > self.prev_path_time + self.delay_time:
                self.prev_path_time = None
                continue
            """
            if self.compareTol() and len(self.targets) > 0:
                self.gTurret.setP0T(self.targets.pop(0))
                q1, q2, toa, f = self.gTurret.inverseKin(print_time=True)
                pathMatrix = self.gTurret.scurvePath(self.actual_configuration,
                                                         np.array([q1, q2]).reshape(2, 1),
                                                         10, 1.5, .05)

                self.dest_configuration = np.array([[pathMatrix[1][pathMatrix.shape[1]-1],
                                                     pathMatrix[2][pathMatrix.shape[1]-1]]]).T

                self.publish(pathMatrix)

            time.sleep(.02)
