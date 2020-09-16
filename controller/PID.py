import math
import numpy as np

class PID(object):
    def __init__(self, kp, kd ,ki):
        self.desired_PID = 0.0
        
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.last_error = 0

    def set_desired_PID(self, speed):
        self.desired_PID = speed

    def update_PID(self, now, fps):
        dt = 1/fps

        error = self.desired_PID - now

        derivative = (error - self.last_error)/dt

        integral = integral + error * dt

        output = self.kp * error + self.kd * derivative + self.ki * integral

        self.last_error = error

        return output

   

class Robot_PID(object):

    def __init__(self, robot):
        self.robot = robot
        self.game = self.robot.game

        self.desired = np.array([0,0])

        self.linear_pid = PID(1,0,0)
        self.angular_pid = PID(1,0,0)

    def update(self):
        linear_speed, angular_speed = self.robot._get_differential_robot_speeds(self.robot.vx, self.robot.vy, self.robot.theta)
        linear_desired, angular_desired = self.robot._get_desired_differential_robot_speeds(self.desired[0],self.desired[1], self.robot.theta)
        vl, va = self.update_Speed(linear_desired,angular_desired,linear_speed, angular_speed)

        acc_left  = vl + va
        acc_right = vl - vl

        power_left = power_left + acc_left * (1/self.game._fps)
        power_right = power_right + acc_right * (1/self.game._fps)

        return power_left , power_right



    def set_desired(self, vector):
        self.desired = vector




    def update_Speed(self, linear_desired, angular_desired, now_linear, now_angular):
        self.linear_desired = linear_desired
        self.angular_desired = angular_desired

        self.linear_pid.set_desired_PID(linear_desired)
        self.angular_pid.set_desired_PID(angular_desired)

        vl = self.linear_pid.update_PID(now_linear,self.game._fps)
        va = self.angular_pid.update_PID(now_angular,self.game._fps)

        return vl, va
