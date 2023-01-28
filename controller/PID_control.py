import math
import numpy as np
from live_plot import Writer, Parameter
from commons.math import speed_to_power

def angle_adjustment(angle):
        """Adjust angle of the robot when objective is "behind" the robot"""
        phi = angle % math.radians(360)
        if phi > math.radians(180):
            phi = phi - math.radians(360)

        return phi

class PID_W_control(object):

    CONSTANTS = {
        'simulation': {
            # Control params
            'K_RHO': 500, # Linear speed gain
            # PID of angular speed
            'KP': 3, # Proportional gain of w (angular speed), respecting the stability condition: K_RHO > 0 and KP > K_RHO
            'KD': 100, # Derivative gain of w
            'KI': 5, # Integral gain of w
            # Max speeds for the robot
            'V_MAX': 40, # linear speed
            'W_MAX': math.radians(7200), # angular speed rad/s
            'V_MIN': 80
        },
        'real_life': {
            # Control params
            'K_RHO': .05, # Linear speed gain
            # PID of angular speed
            'KP': -350, # Proportional gain of w (angular speed), respecting the stability condition: K_RHO > 0 and KP > K_RHO
            'KD': 0, # Derivative gain of w
            'KI': 0, # Integral gain of w
            # Max speeds for the robot
            'V_MAX': 130, # linear speed
            'W_MAX': 550, # angular speed rad/s
            'V_MIN': 80
        }
    }

    def __init__(self, robot, default_fps=60):
        self.vision = robot.game.vision
        self.field_w, self.field_h = robot.game.field.get_dimensions()
        self.robot = robot
        self.desired = [0, 0]
        self.environment = robot.game.environment

        self.l = self.robot.dimensions.get('L')/2 # half_distance_between_robot_wheels
        self.R = self.robot.dimensions.get('R')   # radius of the wheel

        self.default_fps = default_fps
        self.dt = 1/self.default_fps

        self.__dict__.update( self.CONSTANTS.get(self.environment) )

        # PID params for error
        self.dif_alpha = 0 # diferential param
        self.int_alpha = 0 # integral param
        self.alpha_old = 0 # stores previous iteration alpha

        self.lp = [0, 0]

        self.robot_writer = Writer('robot',
                                {'x': 'FLOAT',
                                    'y': 'FLOAT'})
    
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
        rhox = self.robot.x - self.lp[0]
        rhoy = self.robot.y - self.lp[1]
        rho = math.sqrt((rhox**2 + rhoy**2))

        # GAMMA robot's position angle to the objetive
        gamma = angle_adjustment(math.atan2(D_y, D_x))

        # ALPHA angle between the front of the robot and the objective
        alpha = angle_adjustment(gamma - self.robot.theta)

        """Calculate the parameters of PID control"""
        self._update_fps()
        self.dif_alpha = (alpha - self.alpha_old) / self.dt # Difentential of alpha
        self.int_alpha = self.int_alpha + alpha
            
        """Linear speed (v)"""
        v = self.V_MAX

        """Angular speed (w)"""
        w = self.KP * alpha + self.KI * self.int_alpha + self.KD * self.dif_alpha
        w = np.sign(w) * min(abs(w), self.W_MAX)
        
        self.alpha_old = alpha

        self.robot_writer.write([self.robot.x, self.robot.y])

        return v, w

    def update(self):
        v, w = self._update()

        if self.environment == 'simulation':
            powers = speed_to_power(v, w)
            return tuple(np.dot(1000, powers))
        
        return v, w

class PID_control(PID_W_control):

    def update(self):
        _, w = super()._update()

        v = self.V_MAX

        if self.environment == 'simulation':
            powers = speed_to_power(v, w)
            return tuple(np.dot(1000, powers))

        return v, w
