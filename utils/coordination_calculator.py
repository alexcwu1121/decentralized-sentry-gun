import numpy as np
from math import *
import sympy as sp

def xRot(theta):
    return np.array([[1, 0, 0],
                   [0, cos(theta), -sin(theta)],
                   [0, sin(theta), cos(theta)]])


def yRot(theta):
    return np.array([[cos(theta), 0, -sin(theta)],
                   [0, 1, 0],
                   [-sin(theta), 0, cos(theta)]])


def zRot(theta):
    return np.array([[cos(theta), -sin(theta), 0],
                   [sin(theta), cos(theta), 0],
                   [0, 0, 1]])

def xRot_s(theta):
    return sp.Matrix([[1, 0, 0],
                   [0, sp.cos(theta), -sp.sin(theta)],
                   [0, sp.sin(theta), sp.cos(theta)]])


def yRot_s(theta):
    return sp.Matrix([[sp.cos(theta), 0, -sp.sin(theta)],
                   [0, 1, 0],
                   [-sp.sin(theta), 0, sp.cos(theta)]])


def zRot_s(theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0],
                   [sp.sin(theta), sp.cos(theta), 0],
                   [0, 0, 1]])

# enter the axes and the angles to calculate the final rotation matrix
def Rodriguez(alpha, theta, gamma):
    #print(rot(axis1, theta1), "\n", rot(axis2, theta2), "\n", rot(axis3, theta3))
    return zRot(gamma) @ yRot(theta) @ xRot(alpha)

# Could overload for np or sp matrices, but only works on sp for now
def skewsym(p):
    return sp.Matrix([[0, -p[2, 0], p[1, 0]],
                     [p[2, 0], 0, -p[0, 0]],
                     [-p[1, 0], p[0, 0], 0]])

# Could overload for np or sp matrices, but only works on sp for now
def phi(R, p):
    phi_r = sp.zeros(6)
    phi_r[0:3, 0:3] = R
    phi_r[3:6, 3:6] = R
    phi_r[0:3, 3:6] = -R * skewsym(p)
    return phi_r

# enter the unit vector the arm rotate about to calculate the rotation matrix
def rotation2(x, y, z, theta):
    length = x**2 + y**2 + z**2
    if abs(length - 1) / (0.5 * (length + 1)) > 0.001:
        return False
    R = np.array([[cos(theta)+(x**2)*(1-cos(theta)), x*y*(1-cos(theta))-z*sin(theta), x*z*(1-cos(theta))+y*sin(theta)],
                  [y*x*(1-cos(theta))+z*sin(theta), cos(theta)+(y**2)*(1-cos(theta)), y*z*(1-cos(theta))-x*sin(theta)],
                  [z*x*(1-cos(theta))-y*sin(theta), z*y*(1-cos(theta))+x*sin(theta), cos(theta)+(z**2)*(1-cos(theta))]])
    return R


if __name__ == '__main__':
    # p is the list of p01 to pnT
    p = [np.array([[1], [1], [1]]), np.array([[1], [1], [1]]), np.array([[1], [1], [1]]), np.array([[1], [1], [1]])]
    # R is the list of rotation matrices len(R) = len(p)-1
    R = [rotation2(0, 0, 1, pi), rotation2(0, 1, 0, pi), rotation2(0, 0, 1, pi)]

    # print(p[0])
    # print(np.matmul(R[0], p[1]))
    # print(np.matmul(np.matmul(R[0], R[1]), p[2]))
    p0T = p[0]
    for i in range(len(p) - 1):
        temp = R[0]
        for j in range(i):
            temp = np.matmul(temp, R[j + 1])
        p0T = p0T + np.matmul(temp, p[i + 1])
    print(p0T)




