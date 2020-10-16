import math


from commons.math import angle_between
import numpy as np


"""
Essa variavel experimental serve para converter o resultado do LQR
para um valor coerente a velocidade desejada em m/s
"""
EXPERIMENTAL_SPEED_CONSTANT = 2678.57

class SimpleLQR(object):
    def __init__(self, robot, l=0.20):
        self.desired = np.array([0, 0])
        self.robot = robot

        self.l = l
        self.L = self.robot.dimensions.get('L')
        self.R = self.robot.dimensions.get('R')

    def set_desired(self, vector):

        self.desired = (self.robot.x + vector[0] * EXPERIMENTAL_SPEED_CONSTANT, self.robot.y + vector[1] * EXPERIMENTAL_SPEED_CONSTANT)

    def update(self):
        n = (1/self.l)

        inverted = False
        theta = self.robot.theta
        robot_to_target = self.desired

        v = self.desired[0] * math.cos(-theta) - self.desired[1] * math.sin(-theta)
        w = n * (self.desired[0] * math.sin(-theta) + self.desired[1] * math.cos(-theta))

        pwr_left = (2 * v - w * self.L)/2 * self.R
        pwr_right = (2 * v + w * self.L)/2 * self.R

        return pwr_left, pwr_right