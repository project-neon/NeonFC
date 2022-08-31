import math
import numpy as np

def angle_adjustment(angle):
        phi = angle % math.radians(360)
        if phi > math.radians(180):
            phi = phi - math.radians(360)

class PID_control(object):
    def __init__(self, match, robot, desired_point, fps):
        self.field_w, self.field_h = m.game.field.get_dimensions()
        self.robot = robot
        self.desired = desired_point

        self.l = self.robot.dimensions.get('L')/2 # half_distance_between_robot_wheels
        self.R = self.robot.dimensions.get('R')   # radius of the wheel

        self.dt = 1/fps

        # Control params
        self.K_RHO = 1 # Linear speed gain

        # PID of angular speed
        self.KP = 1.2 # Proportional gain of w (angular speed), respecting the stability condition: K_RHO > 0 and KP > K_RHO
        self.KI = 0.5 # Integral gain of w 
        self.KD = 0 # Derivative gain of w

        # PID params for error
        self.dif_aplha = 0 # diferential param
        self.int_aplha = 0 # integral param
        self.aplha_old = 0 # stores previous iteration alpha

        # Max speeds for the robot
        self.v_max = 1 # linear speed 
        self.w_max = math.radians(300) # angular speed rad/s