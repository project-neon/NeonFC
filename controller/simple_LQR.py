import math
import numpy as np

class SimpleLQR(object):
    def __init__(self, robot, l=0.20):
        self.desired = np.array([0, 0])
        self.robot = robot

        self.l = l
        self.L = self.robot.dimensions.get('L')
        self.R = self.robot.dimensions.get('R')

    def set_desired(self, vector):
        self.desired = vector

    def update(self):
        n = (1/self.l)

        v = self.desired[0] * math.cos(-self.robot.theta) - self.desired[1] * math.sin(-self.robot.theta)
        w = n * (self.desired[0] * math.sin(-self.robot.theta) + self.desired[1] * math.cos(-self.robot.theta))

        pwr_left = (2 * v - w * self.L)/2 * self.R
        pwr_right = (2 * v + w * self.L)/2 * self.R

        return pwr_left, pwr_right