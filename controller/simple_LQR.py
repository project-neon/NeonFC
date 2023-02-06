import math
import numpy as np
import numpy.linalg as la
from commons.math import speed_to_power
 
def py_ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)

"""
Essa variavel experimental serve para converter o resultado do LQR
para um valor coerente a velocidade desejada em m/s
"""
EXPERIMENTAL_SPEED_CONSTANT = 7000

class SimpleLQR(object):
    def __init__(self, robot, l=0.025):
        self.desired = np.array([0, 0])
        self.robot = robot
        self.environment = robot.game.environment

        self.l = l
        self.L = self.robot.dimensions.get('L')
        self.R = self.robot.dimensions.get('R')
        self.inverted = False
    
    def change_orientation(self):
        self.inverted = not self.inverted

    def set_desired(self, vector):

        self.desired = (self.robot.x + vector[0] * EXPERIMENTAL_SPEED_CONSTANT, self.robot.y + vector[1] * EXPERIMENTAL_SPEED_CONSTANT)

    def update(self):
        n = (1/self.l)

        theta = self.robot.theta
        if self.inverted:
            theta = self.robot.theta - math.pi

        v = self.desired[0] * math.cos(-theta) - self.desired[1] * math.sin(-theta)
        w = n * (self.desired[0] * math.sin(-theta) + self.desired[1] * math.cos(-theta))

        if self.environment == 'simulation':
            powers = speed_to_power(v, w, self.L, self.R)
            if self.inverted:
                return tuple(np.dot(-1, powers))
            else:
                return powers

        linear = v*self.R
        angular = self.R*(w*self.L)/2

        if self.inverted:
            linear = -linear

        return angular, linear

class TwoSidesLQR(object):
    def __init__(self, robot, l=0.010):
        self.desired = np.array([0, 0])
        self.robot = robot
        self.environment = robot.game.environment

        self.l = l
        self.L = self.robot.dimensions.get('L')
        self.R = self.robot.dimensions.get('R')

    def set_desired(self, vector):

        self.desired = (self.robot.x + vector[0] * EXPERIMENTAL_SPEED_CONSTANT, self.robot.y + vector[1] * EXPERIMENTAL_SPEED_CONSTANT)

    def update(self):
        n = (1/self.l)

        A = np.array(self.desired)
        B = np.array([math.cos(self.robot.theta), math.sin(self.robot.theta)])
        between = py_ang(A, B)

        if (between > math.pi/2):
            theta = self.robot.theta - math.pi
            #print("LADO 1")
        else:
            theta = self.robot.theta
            #print("LADO 2")

        v = self.desired[0] * math.cos(-theta) - self.desired[1] * math.sin(-theta)
        w = n * (self.desired[0] * math.sin(-theta) + self.desired[1] * math.cos(-theta))

        if self.environment == 'simulation':
            powers = speed_to_power(v, w, self.L, self.R)

            if (between > math.pi/2):
                # this multiplies -1 to both elements of 'powers' and return it as a tuple
                return tuple(np.dot(-1, powers))
            else:
                return powers

        linear = v*self.R
        angular = self.R*(w*self.L)/2

        if (between > math.pi/2):
            return -linear, -angular
        return linear, -angular 
