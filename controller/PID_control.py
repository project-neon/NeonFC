import math
import numpy as np

def angle_adjustment(angle):
        """Adjust angle of the robot when objective is "behind" the robot"""
        phi = angle % math.radians(360)
        if phi > math.radians(180):
            phi = phi - math.radians(360)

class PID_control(object):
    def __init__(self, match, robot, desired_point, fps):
        self.field_w, self.field_h = match.game.field.get_dimensions()
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
        self.dif_alpha = 0 # diferential param
        self.int_alpha = 0 # integral param
        self.alpha_old = 0 # stores previous iteration alpha

        # Max speeds for the robot
        self.v_max = 1 # linear speed 
        self.w_max = math.radians(300) # angular speed rad/s

    def update(self):
        # Params calculation
        # Feedback errors
        D_x =  self.desired[0] - self.robot.x
        D_y =  self.desired[1] - self.robot.y

        # RHO distance of the robot to the objective
        rho = math.sqrt((D_x**2 + D_y**2))

        # GAMMA robot's position angle to the objetive
        gamma = angle_adjustment(math.atan2(D_y, D_x))

        # ALPHA angle between the front of the robot and the objective
        alpha = angle_adjustment(gamma - self.robot.theta)


        """Calculate the parameters of PID control"""
        #self.dif_alpha = alpha - self.alpha_old / self.dt # Difentential of alpha
        self.dif_alpha = alpha - self.alpha_old # Difentential of alpha
        self.int_alpha = self.int_alpha + alpha # 

        """Linear speed (v)"""
        v = self.v_max if [self.robot.x, self.robot.y] < [self.desired[0], self.desired[1]] else min(self.K_RHO*rho, self.v_max)

        """Objective behind the robot"""
        if(abs(alpha) > math.radians(math.pi/2)):
            v = -v
            alpha = angle_adjustment(alpha+math.radians(math.pi))

        """Angular speed (w)"""
        w = self.KP * alpha + self.KI * self.int_alpha + self.KD * self.dif_alpha
        w = np.sign(w) * min(abs(w), self.w_max)
        
        """Wheel power calculation"""
        pwr_left = (2 * v - w * self.l)/2 * self.R
        pwr_right = (2 * v + w * self.l)/2 * self.R

        return pwr_left, pwr_right
