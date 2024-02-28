import math
import numpy as np
from commons.math import speed_to_power
from api import Parameter

"""
Angle based controller
reffering to soccer robotics
"""


class UniController(object):
    """
    An implementation of the Uni controller specified on the soccer robotics article

    The UniController will receive a desired angle for the robot current position, the desired angle for the position in
    front of the robot and along with the current robot angle it will calculate the robot angular and linear speed. it
    can also be set through the control_speed parameter to slow down when near the target position.

    Attributes
    ----------
    K_P : float
        the linear Kp value
    control_speed : bool
        whether the controller will use the P control for linear speed
    V_M : int
        the maximum linear speed
    K_W : float
        a positive constant
    R_M : int
        the maximum turn (v*w)
    target: tuple[float, float]
        the target position used for calculating the linear speed P
    """

    CONSTANTS = {
        'simulation': {
            'V_M': 100,
            'R_M': 3 * 100,  # 3 * V_M
            'K_W': 90,
            'K_P': 10
        },
        'real_life': {
            'V_M': 0.5,
            'R_M': 0.44,  # 20 * V_M
            'K_W': 3.5,  # 313,
            'K_P': 1
        }
    }

    def __init__(self, robot, control_speed=False):
        self.robot = robot
        self.environment = robot.game.environment
        self.match = robot.game.match
        self.L = self.robot.dimensions.get("L")  # m
        self.R = self.robot.dimensions.get("R")  # m

        self.__dict__.update(self.CONSTANTS.get(self.environment))

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

        self.parameter = self.match.parameter
        self.K_P = self.parameter.kp
        self.K_W = self.parameter.kw
        self.R_M = self.parameter.rm
        self.V_M = self.parameter.vm
        
        self.target = [1.5, 0.65]

    def control(self):
        """
        x, y, q: robot's posture (x position, t position, robot's angle)
        returns: v and w, linear and angular speed, respectively
        """

        self.update_parameters()

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

        return v, w

    def set_desired(self, desired):
        """
        Defines the target angles

            Parameters
            ----------
                desired (tuple[float, float]): the desired angle in the current position and in the desired angle in
                                               front of the robot
        """
        self.theta_d = desired[0]
        self.theta_f = desired[1]

    def update(self):
        v, w = self.control()

        if self.environment == 'simulation':
            return tuple(np.dot(250, speed_to_power(v, w, self.L, self.R)))

        # w = w if abs(w) < 4 else 4 * w / abs(w)
        return -v, -w

    def update_parameters(self):
        if self.K_P != self.parameter.kp:
            self.K_P == self.parameter.kp
        if self.K_W != self.parameter.kw:
            self.K_W == self.parameter.kw
        if self.R_M != self.parameter.rm:
            self.R_M == self.parameter.rm
        if self.V_M != self.parameter.vm:
            self.V_M == self.parameter.vm