import numpy as np
import sympy as sp
from Entity import Line, Point, DottedLine
from Geometry import Geometry
from Turret import Turret
from utils.coordination_calculator import *


class CameraTurret(Turret):
    def __init__(self, targets = None, q1=0, q2=0, pOffset=np.array([-10, 0, 0]).reshape(3, 1),
                 p12=np.array([0, 74, 60]).reshape(3, 1),
                 orig=np.array([100, 50, 0]).reshape(3, 1)):
        super().__init__(q1, q2, pOffset, p12, orig)
        self.targets = targets
        self.num_q = 2

        # Only need to run initSimCamera for simulations
        self.initSimCamera()

    def initSimCamera(self):
        self.focal_length = 100
        self.view_angle = 0.959931

    def getEntities(self):
        R01 = zRot(self.q1_given)
        R12 = yRot(self.q2_given)

        p_1 = self.orig + R01 @ self.p12
        p_2 = p_1 + R01 @ R12 @ self.pOffset

        # p_1alpha, p_1beta, p_1gamma are for visualization purposes only
        p_1alpha = self.orig + R01 @ np.array([0, 0, self.p12[2][0]]).reshape(3, 1)
        p_1beta = p_1alpha + R01 @ np.array([0, self.p12[1][0], 0]).reshape(3, 1)
        p_1gamma = p_1beta + R01 @ np.array([self.p12[0][0], 0, 0]).reshape(3, 1)

        #print(self.targetsInView())

        # Visualization of camera plane
        camera_plane_entities = self.cameraPlaneBounds(p_2)

        #target_p = self.orig if self.targets is None else self.targets[0].pos
        lens_pos = self.representTarget() + self.orig
        # target list can be changed to include only targets in view
        target_p = self.getTargetLinks(lens_pos, self.targets)
        target_rep = self.representTarget(target_p[0]) + self.orig

        return [Line(np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), p_2, "black", 2),
                Line(np.array([0, p_2[1][0], 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),
                Line(np.array([p_2[0][0], 0, 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),
                Line(p_1, p_2, "orange", 8),
                DottedLine(p_1, p_1 + 5*(p_2 - p_1), "black", 2, 10),
                Line(self.orig, p_1alpha, "orange", 8),
                Line(p_1alpha, p_1beta, "orange", 8),
                Line(p_1beta, p_1gamma, "orange", 8),
                DottedLine(self.orig, target_rep, "red", 2, 10),
                DottedLine(lens_pos, target_rep, "blue", 2, 10),
                Point(self.orig, 'black', 8),
                Point(p_1, 'black', 8),
                Point(p_2, 'black', 8),
                ] +  camera_plane_entities

    # Function only relevant for simulations, so take focal length and view angle as parameters
    def cameraPlaneBounds(self, p_2):
        # Typical webcam view angles between 55 (0.959931 rad) and 65 degrees (1.13446 rad)
        R01 = zRot(self.q1_given)
        R12 = yRot(self.q2_given)

        corner_list = []
        corner_list.append(p_2 + R01 @ R12 @ np.array([-self.focal_length,
                                                       self.focal_length * np.tan(self.view_angle/2),
                                                       self.focal_length * np.tan(self.view_angle/2)]).reshape(3, 1))
        corner_list.append(p_2 + R01 @ R12 @ np.array([-self.focal_length,
                                                       -self.focal_length * np.tan(self.view_angle / 2),
                                                       self.focal_length * np.tan(self.view_angle / 2)]).reshape(3, 1))
        corner_list.append(p_2 + R01 @ R12 @ np.array([-self.focal_length,
                                                      -self.focal_length * np.tan(self.view_angle / 2),
                                                       -self.focal_length * np.tan(self.view_angle / 2)]).reshape(3, 1))
        corner_list.append(p_2 + R01 @ R12 @ np.array([-self.focal_length,
                                                       self.focal_length * np.tan(self.view_angle / 2),
                                                       -self.focal_length * np.tan(self.view_angle / 2)]).reshape(3, 1))

        entities = []
        for i in range(len(corner_list)):
            entities.append(Line(corner_list[i], p_2, "yellow", 2))
            if i < len(corner_list) - 1:
                entities.append(Line(corner_list[i], corner_list[i+1], "yellow", 2))
            else:
                entities.append(Line(corner_list[i], corner_list[0], "yellow", 2))

        return entities

    def targetsInView(self):
        # Check all given targets and return a list of targets that are in view
        # Compute getTargetLinks
        # Compute angle between x/y and z axes and check if they lie within view range
        R01 = zRot(self.q1_given)
        R12 = yRot(self.q2_given)

        p_1 = self.orig + R01 @ self.p12
        p_2 = p_1 + R01 @ R12 @ self.pOffset

        t_links = self.getTargetLinks(p_2, self.targets)
        in_view = dict()

        for t_link, target in zip(t_links, self.targets):
            tvec = zRot(np.pi) @ yRot(np.pi/2) @ t_link
            if tvec[2][0] < 0:
                continue
            if abs(np.arctan2(tvec[0][0], tvec[2][0])) < self.view_angle/2 and \
                    abs(np.arctan2(tvec[1][0], tvec[2][0])) < self.view_angle/2:
                in_view[target.getId()] = target

        return in_view

    def getTargetLinks(self, lens_pos, targets):
        t_links = []
        for target in targets:
            # Simulate tvecs coming from Aruco by representing distances between
            # the camera lens and each target in the camera lens' frame
            c_t = target.pos - lens_pos

            # (c_t)c = R20(c_t)o
            R01 = zRot(self.q1_given)
            R12 = yRot(self.q2_given)

            R02 = R01 @ R12
            R20 = R02.transpose()
            c_tc = R20 @ c_t

            # Set axes relative to Aruco camera axis
            # Produces a 'tvec', a pinhole camera translation vector given by Aruco
            #c_tc = zRot(np.pi) @ yRot(np.pi/2) @ c_tc

            # TODO Rotate axes back to camera frame.
            # Rotates tvec to camera frame. Final offset sent to POE

            t_links.append(c_tc)

        return t_links

    def representTarget(self, distance=np.array([0, 0, 0]).reshape(3, 1)):
        # Define POE parameters
        # h1 = [0, 0, 1]T or ez
        # h2 = [1, 0, 0]T or ex
        # p01 = [0, 0, 0]T
        # p12 = self.p12, default case [0, 0, 75]T
        # p2T = self.pOffset + {target vector}
        p01 = np.array([0, 0, 0]).reshape(3, 1)
        p2T = self.pOffset + distance

        # Define rotation matrices
        # e^(h1xq1) = Rz(q1)
        # e^(h2xq2) = Rx(q2)
        R01 = zRot(self.q1_given)
        R12 = yRot(self.q2_given)

        # Define HTMS
        # T01 = |R01    p01|
        #       |0      1  |
        T01 = np.identity(4)
        T01[0:3, 0:3] = R01
        T01[0:3, 3:4] = p01

        # T12 = |R12    p12|
        #       |0      1  |
        T12 = np.identity(4)
        T12[0:3, 0:3] = R12
        T12[0:3, 3:4] = self.p12

        # T2T = |I      p2T|
        #       |0      1  |
        T2T = np.identity(4)
        T2T[0:3, 3:4] = p2T

        target = T01 @ T12 @ T2T

        return target[0:3, 3:4]

    # Returns a path matrix representing the entire sweep
    # sinusoidal sweep
    # in position space: y = 100*sin(pi*z/4)
    # in configuration space: q1 = a*sin(b*(q2-c))+d
    # (0, 0) is camera turret at zero configuration
    # increasing t_elapse decreases speed of camera
    def sweepPath(self, t_elapse, time_step = 0.1, q1_range = (-np.pi, np.pi), q2_range = (-np.pi, np.pi)):
        r1 = getRange(q1_range)
        r2 = getRange(q2_range)
        a = r1
        b = 2.5 * np.pi / r2
        c = q2_range[1] - r2
        d = q1_range[1] - r1
        
        # Find number of time steps and allocate path matrix at size
        num_steps = np.ceil(t_elapse / time_step).astype('int')
        period = 2 * r2 / num_steps
        t = q2_range[0]   # t initialized to pi because of zero configuration offest from yz-plane
        q_steps = np.zeros([self.num_q+1, num_steps+1])

        for i in range(num_steps+1):
            # parametrize path to q2 = t, q1 = a*sin(b*(t-c))+d with zero_config offset
            q_step = np.array([(a*np.sin(b*(t-c))+d), t]).reshape(2, 1)
            q_steps[0, i:i+1] = (i * time_step)
            q_steps[1:3,i:i+1] = q_step
            t += period

        return q_steps

    def inverseKin(self, targetPos):
        # MATLAB code from Robotics class used as reference

        # Solve for both q1 and q2 using subproblem 2
        # P0T = R01 @ P12 + R01 @ R12 @ (P23 + P3T)     --- use this
        # R01^T @ P0T = P12 + R12 @ (P23 + P3T)
        # k1 = -ez, p1 = p0T - p12 = p2T_target, k2 = ey, p2 = p2T

        R01T = zRot_s(-self.q1)
        R12 = yRot_s(self.q2)

        # self.pOffset = p2T where T is at end effector of turret
        p2T_f = targetPos - self.p12
        p2T_f = p2T_f / np.linalg.norm(p2T_f)
        p2T_f = p2T_f * np.linalg.norm(self.pOffset)
        print(p2T_f / np.linalg.norm(p2T_f))

        #  0 -1  0
        #  0  0  1
        # -1  0  0

        pk1 = -p2T_f[2][0]
        pk2 = -self.pOffset[1][0]
        a = np.array([pk1, pk2]).reshape(2, 1)
        cond = np.linalg.norm(p2T_f)**2 - np.linalg.norm(a)**2
        v1 = np.array([cond**0.5, pk2, -pk1]).reshape(3, 1)
        
        pp11 = p2T_f - np.array([0, 0, -pk1]).reshape(3, 1)
        pp21 = v1 - np.array([0, 0, -pk1]).reshape(3, 1)
        pp12 = self.pOffset - np.array([0, pk2, 0]).reshape(3, 1)
        pp22 = v1 - np.array([0, pk2, 0]).reshape(3, 1)

        numer1 = (pp11 / np.linalg.norm(pp11)) - (pp21 / np.linalg.norm(pp21))
        denom1 = (pp11 / np.linalg.norm(pp11)) + (pp21 / np.linalg.norm(pp21))
        numer2 = (pp12 / np.linalg.norm(pp12)) - (pp22 / np.linalg.norm(pp22))
        denom2 = (pp12 / np.linalg.norm(pp12)) + (pp22 / np.linalg.norm(pp22))

        q1_sol = np.arctan2(np.linalg.norm(numer1), np.linalg.norm(denom1))
        q2_sol = np.arctan2(np.linalg.norm(numer2), np.linalg.norm(denom2))
        #print(q1_sol, q2_sol)

        return np.array([q1_sol, q2_sol]).reshape(2, 1)

# Helper function to get range of an angle
# r = (min, max)
def getRange(r):
    return (r[1] - r[0]) / 2.0

# for testing
if __name__ == "__main__":
    ct = CameraTurret()
    print(ct.sweepPath(10))
