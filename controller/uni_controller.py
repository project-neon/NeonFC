import math
import numpy as np
from commons.math import speed_to_power

"""
Angle based controller
reffering to soccer robotics
"""

class UniController(object):
    
    CONSTANTS = {
        'simulation': {
            'V_M': 100,
            'R_M': 3 * 100, # 3 * V_M
            'K_W': 90,
            'K_P': 5
        },
        'real_life': {
            'V_M': 10000,
            'R_M': 3 * 10000, # 20 * V_M
            'K_W': 270, # 313,
            'K_P': 100
        }
    }

    def __init__(self, robot, control_speed=False):
        self.robot = robot
        self.environment = robot.game.environment
        self.L = self.robot.dimensions.get("L")  # m
        self.R = self.robot.dimensions.get("R")  # m

        self.__dict__.update( self.CONSTANTS.get(self.environment) )

        self.control_speed = control_speed
        self.v1 = 0  # speed limit 1
        self.v2 = 0  # speed limit 2
        self.theta_d = 0
        self.theta_f = 0
        self.dl = 0.000001  # approximate phi_v in m
        self.phi_v = 0
        self.a_phi_v = 0  # absolute value of phi_v
        self.theta_e = 0
        self.a_theta_e = 0  # absolute value of theta_e

        self.target = [1.5, 0.65]

    def control(self):
        """
        x, y, q: robot's posture (x position, t position, robot's angle)
        returns: v and w, linear and angular speed, respectively
        """

        self.phi_v = self.theta_f - self.theta_d

        while self.phi_v > math.pi:
            self.phi_v -= 2 * math.pi
        while self.phi_v < -math.pi:
            self.phi_v += 2 * math.pi

        self.phi_v = self.phi_v / self.dl
        self.a_phi_v = abs(self.phi_v)

        # calculate theta_e
        self.theta_e = self.theta_d - self.robot.theta

        while self.theta_e > math.pi:
            self.theta_e -= 2 * math.pi
        while self.theta_e < -math.pi:
            self.theta_e += 2 * math.pi

        self.a_theta_e = abs(self.theta_e)

        # calculate v
        self.v1 = (2 * self.V_M - self.L * self.K_W * math.sqrt(self.a_theta_e)) / (2 + self.L * self.a_phi_v)

        self.v2 = (math.sqrt(self.a_theta_e * self.K_W ** 2 + 4 * self.R_M * self.a_phi_v) - self.K_W * math.sqrt(
            self.a_theta_e)) \
                  / (2 * self.a_phi_v) if self.a_phi_v > 0 else self.V_M

        self.v3 = self.K_P * ((self.robot.x - self.target[0]) ** 2 + (self.robot.y - self.target[1]) ** 2) ** .5

        if self.control_speed:
            v = min(self.v1, self.v2, self.v3)
        else:
            v = min(self.v1, self.v2)

        # calculate w
        if self.theta_e > 0:
            w = v * self.phi_v + self.K_W * math.sqrt(self.a_theta_e)
        else:
            w = v * self.phi_v - self.K_W * math.sqrt(self.a_theta_e)

        w *= 1.3

        return v, -w

    def set_desired(self, desired):
        self.theta_d = desired[0]
        self.theta_f = desired[1]

    def update(self):
        v, w = self.control()

        if self.environment == 'simulation':
            return tuple(np.dot(250, speed_to_power(v, w, self.L, self.R)))
            
        return v, w
