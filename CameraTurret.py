import numpy as np
import sympy as sp
from Entity import Line, Point, DottedLine
from Geometry import Geometry
from utils.coordination_calculator import xRot, yRot, zRot


class CameraTurret(Geometry):
    def __init__(self, targets = None, q1=0, q2=0, pOffset=np.array([0, 50, 0]).reshape(3, 1),
                 p12=np.array([0, 0, 75]).reshape(3, 1),
                 orig=np.array([100, 50, 0]).reshape(3, 1)):
        super().__init__()
        self.q1 = q1
        self.q2 = q2
        self.pOffset = pOffset
        self.p12 = p12
        self.orig = orig
        self.targets = targets

    def getEntities(self, elapsed_time = 0):
        R01 = zRot(self.q1)
        R12 = xRot(self.q2)

        p_1 = self.orig + R01 @ self.p12
        p_2 = p_1 + R01 @ R12 @ self.pOffset

        #target_p = self.orig if self.targets is None else self.targets[0].pos
        lens_pos = self.representTarget() + self.orig
        target_p = self.getTargetLinks(lens_pos)
        target_rep = self.representTarget(target_p[0]) + self.orig

        print(lens_pos, target_p, target_rep)

        return [Line(np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1), p_2, "black", 2),
                Line(np.array([0, p_2[1][0], 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),
                Line(np.array([p_2[0][0], 0, 0]).reshape(3, 1), np.array([p_2[0][0], p_2[1][0], 0]).reshape(3, 1),
                     "black", 2),

                Line(self.orig, p_1, "orange", 8),
                Line(p_1, p_2, "orange", 8),
                DottedLine(self.orig, target_rep, "red", 2, 10),
                DottedLine(lens_pos, target_rep, "blue", 2, 10),
                Point(self.orig, 'black', 10),
                Point(p_1, 'black', 10),
                Point(p_2, 'black', 10),
                ]

    def zeroConfiguration(self):
        # in the origin frame, so orig = (0, 0, 0)
        return self.p12 + self.pOffset, np.identity(3)

    def getTargetLinks(self, lens_pos):
        t_links = []
        for target in self.targets:
            # Simulate tvecs coming from Aruco by representing distances between
            # the camera lens and each target in the camera lens' frame
            c_t = target.pos - lens_pos

            # (c_t)c = R20(c_t)o
            R01 = zRot(self.q1)
            R12 = xRot(self.q2)

            R02 = R01 @ R12
            R20 = R02.transpose()
            c_tc = R20 @ c_t

            # Set axes relative to Aruco camera axis
            # Produces a 'tvec', a pinhole camera translation vector given by Aruco
            #c_tc = xRot(-np.pi/2) @ yRot(np.pi) @ c_tc

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
        R01 = zRot(self.q1)
        R12 = xRot(self.q2)

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

    def sweepPath(self, init_q = np.array([0, 0]).reshape(2, 1), t_elapse, time_step):
        # sinusoidal sweep: follows y = 100*sin(pi*z/4) path
        # assuming for now that velocity is a constant 50 units/s
        # define (0, 0) as camera turret at zero configuration

        vel = 50
        z = float(t_elapse*vel)
        y = 100*np.sin(np.pi*float(t_elapse*vel)/4)
        # x doesn't change
        # result = self.inverseKin(self.zeroConfiguration() + np.array([0 y z]).reshape(3, 1))

        # use result to get (q1, q2), then create path matrix from init_pos to result angles
        # path matrix will have to stop and reverse when camera motion limit is reached
        # and stop completely when a target is found
        # make timestep very small to ensure motion is along path as much as possible
        # may have to calculate inverse kinematics for every iteration

        num_steps = math.ceil(t_elapse / time_step)
        q_steps = np.zeros([self.num_q, num_steps])
        timestamps = np.zeros([1, num_steps])

    # def inverseKin(self, p0T = self.zeroConfiguration()[0]):
    #     # Arrange kinematic chain into subproblem 4
    #     # P0T = P01 + R12 @ P12 + R12 @ R2T @ P2T
    #     # ezT(P12 + P2T) = exT @ Rz(q1)T @ (P0T - P01)
    #     # Where P2T is pOffset + [0, dh, dv] = [0; l2 + dh; dv]
    #     exT = sp.Matrix([[1, 0, 0]])
    #     Rzq1 = sp.Matrix([[sp.cos(self.q1), -sp.sin(self.q1), 0],
    #                         [sp.sin(self.q1), sp.cos(self.q1), 0],
    #                         [0, 0, 1]])

    #     expr = exT * Rzq1 * (sp.Matrix(p0T) - sp.Matrix(self.p01))

    #     p2T = sp.Matrix(self.pOffset) + sp.Matrix([[0], [self.dh], [self.dv]])
    #     d = exT * (sp.Matrix(self.p12) + p2T)

    #     # Solve kinematic chain for q1. Only ever one solution.
    #     q1_sol = sp.solve(expr - d, self.q1)[0][0]

    #     # Substitute q1 into kinematic chain and solve for q2 using subproblem 3
    #     # POT - P01 - R12 @ P12 = R12 @ R2T @ P2T
    #     # R12T @ (POT - P01 - R12 @ P12) = R2T @ P2T
    #     # |R12T @ (POT - P01 - R12 @ P12)| = |R2T @ P2T|
    #     # lhs_norm = rhs_norm
    #     #
    #     # Since system has an extra t variable due to trajectory, use z relation

    #     lhs = Rzq1.T * (sp.Matrix(p0T) - sp.Matrix(self.p01) - Rzq1 * sp.Matrix(self.p12))
    #     lhs_val = lhs.subs(self.q1, q1_sol)

    #     lhs_norm = (lhs_val[0]**2 + lhs_val[1]**2 + lhs_val[2]**2)**.5
    #     lhs_z = lhs_val[2]

    #     Rxq2 = sp.Matrix([[1, 0, 0],
    #                      [0, sp.cos(self.q2), -sp.sin(self.q2)],
    #                      [0, sp.sin(self.q2), sp.cos(self.q2)]])

    #     rhs = Rxq2 * p2T
    #     rhs_norm = (rhs[0]**2 + rhs[1]**2 + rhs[2]**2)**.5
    #     rhs_z = rhs[2]

    #     esys = sp.Matrix([[lhs_norm - rhs_norm], [lhs_z - rhs_z]])

    #     over_sol = sp.nsolve((esys), [self.q2, self.t], [0, 0], modules=['mpmath'])

    #     return -q1_sol, over_sol[0], over_sol[1], (self.p01 + Rzq1*self.p12 + Rzq1*Rxq2*p2T).subs([[self.q1, -q1_sol],
    #                                                                                                [self.q2, over_sol[0]]])



    # Unnecessary for cameraTurret for now
    def scurvePath(self):
        pass