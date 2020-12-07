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
        self.tol = .05

        # Remember previous state to track state transitions
        self.prev_state = False

        # queue of P0Ts to process
        self.targets = []

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
        # If there is no destination, comparison fails
        if not self.dest_configuration:
            return False

        # If there is a destination, see if configs matche
        #for q, q_prime in zip(self.actual_configuration, self.dest_configuration):
        for i in range(self.actual_configuration.shape[0]):
            if (self.actual_configuration[i][0] > self.dest_configuration[i][0] + self.tol or
                    self.actual_configuration[i][0] < self.dest_configuration[i][0] - self.tol):
                return False

        # If the configs match, then there is a guaranteed state change. Set destination to none, now
        # that it has been reached.
        self.dest_configuration = None

        # Delay for as long as the gun takes to wind up and shoot
        # For real demo, about 6 seconds
        time.sleep(2)

        print("pass")
        print(self.dest_configuration)

        # Allow another path matrix to be sent
        return True

    def run(self, is_sim):
        """
        Checks the configuration of gun turret and the target, if a target is received, calculate the path matrix and
        delete the target to wait for the next one
        """
        while True:
            self.receive()
            if self.compareTol and len(self.targets) > 0:
                self.gTurret.setP0T(self.targets.pop(0))
                q1, q2, toa, f = self.gTurret.inverseKin(print_time=True)
                pathMatrix = self.gTurret.scurvePath(self.actual_configuration,
                                                         np.array([q1, q2]).reshape(2, 1),
                                                         10, 1.5, .05)

                self.target = False
                self.publish(pathMatrix)
            time.sleep(.02)
