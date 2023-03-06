import math
import numpy as np
from commons.math import speed_to_power, angle_between


def angle_adjustment(angle):
    """Adjust angle of the robot when objective is "behind" the robot"""
    phi = angle % math.radians(360)
    if phi > math.radians(180):
        phi = phi - math.radians(360)

    return phi


class PID_control(object):
    """
    An implementation of the PID controller on linear and angular speed

    The PID_control will receive a target position (x, y) and calculate the angular speed needed to reach the target
    angle (ensure that the robot angle is in the direction of the target) using the PID algorithm. It will also
    calculate the linear speed based on the distance from the robot and the target using a P algorithm.

    Attributes
    ----------
    K_RHO : float
        the linear Kp value
    KP : float
        the angular Kp value
    KD : float
        the angular Kd value
    KI : float
        the angular Ki value
    V_MAX : int
        the maximum linear speed
    V_MIN : int
        the minimum linear speed
    W_MAX : int
        the maximum angular speed
    TWO_SIDES: bool
        whether the controller will consider the robot as having two equal sides or only one
    """

    CONSTANTS = {
        'simulation': {
            # Control params
            'K_RHO': 100, # Linear speed gain
            # PID of angular speed
            'KP': 60, # Proportional gain of w (angular speed), respecting the stability condition: K_RHO > 0 and KP > K_RHO
            'KD': 0, # Derivative gain of w
            'KI': 0, # Integral gain of w
            # Max speeds for the robot
            'V_MAX': 40, # linear speed
            'W_MAX': math.radians(7200), # angular speed rad/s
            'V_MIN': 80,

            'TWO_SIDES': True
        },
        'real_life': {
            # Control params
            'K_RHO': 500, # Linear speed gain
            # PID of angular speed
            'KP': -1000, # Proportional gain of w (angular speed), respecting the stability condition: K_RHO > 0 and KP > K_RHO
            'KD': 0, # Derivative gain of w
            'KI': 0, # Integral gain of w
            # Max speeds for the robot
            'V_MAX': 130, # linear speed
            'W_MAX': 550, # angular speed rad/s
            'V_MIN': 20,

            'TWO_SIDES': True
        }
    }

    def __init__(self, robot, default_fps=60, **kwargs):
        self.vision = robot.game.vision
        self.field_w, self.field_h = robot.game.field.get_dimensions()
        self.robot = robot
        self.desired = [0, 0]
        self.environment = robot.game.environment

        self.l = self.robot.dimensions.get('L')/2 # half_distance_between_robot_wheels
        self.R = self.robot.dimensions.get('R')   # radius of the wheel

        self.default_fps = default_fps
        self.dt = 1/self.default_fps

        self.__dict__.update(self.CONSTANTS.get(self.environment))
        self.__dict__.update(kwargs)

        # PID params for error
        self.dif_alpha = 0 # diferential param
        self.int_alpha = 0 # integral param
        self.alpha_old = 0 # stores previous iteration alpha

    def set_desired(self, vector):
        self.desired = vector

    def _update_fps(self):
        if self.vision._fps > 0:
            self.dt = 1/self.vision._fps
        else:
            self.dt = 1/self.default_fps

    def _update(self):
        # Params calculation
        # Feedback errors
        D_x = self.desired[0] - self.robot.x
        D_y = self.desired[1] - self.robot.y

        # RHO distance of the robot to the objective
        rho = math.sqrt((D_x**2 + D_y**2))

        # GAMMA robot's position angle to the objetive
        gamma = angle_adjustment(math.atan2(D_y, D_x))
        # ALPHA angle between the front of the robot and the objective
        alpha = angle_adjustment(gamma - self.robot.theta)

        """Calculate the parameters of PID control"""
        self._update_fps()
        self.dif_alpha = (alpha - self.alpha_old) / self.dt # Difentential of alpha
        self.int_alpha = self.int_alpha + alpha

        """Linear speed (v)"""
        v = max(self.V_MIN, min(self.V_MAX, rho*self.K_RHO))

        if (abs(alpha) > math.pi / 2) and self.TWO_SIDES:
            v = -v
            alpha = angle_adjustment(alpha - math.pi)

        """Angular speed (w)"""
        w = self.KP * alpha + self.KI * self.int_alpha + self.KD * self.dif_alpha
        w = np.sign(w) * min(abs(w), self.W_MAX)

        self.alpha_old = alpha

        return v, w

    def update(self):
        v, w = self._update()

        if self.environment == 'simulation':
            powers = speed_to_power(v, w, self.l, self.R)
            return tuple(np.dot(1000, powers))

        return v, w


class PID_W_control(PID_control):

    def update(self):
        v, w = super()._update()

        v = np.sign(v) * self.V_MAX

        if self.environment == 'simulation':
            powers = speed_to_power(v, w, self.l, self.R)
            return tuple(np.dot(1000, powers))

        return v, w
