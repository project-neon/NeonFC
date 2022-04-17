import api
import math
import numpy as np

class PID(object):
    def __init__(self, kp, kd ,ki, _ilimit=1000):
        self.desired_PID = 0.0

        self.kp = kp
        self.kd = kd
        self.ki = ki

        self._ilimit = _ilimit

        self.last_error = 0
        self.integral = 0

    def set_desired_PID(self, speed):
        self.desired_PID = speed

    def update_PID(self, now, fps):
        dt = 1/fps

        error = self.desired_PID - now

        self.integral = min(self._ilimit, max(-self._ilimit, self.integral))

        derivative = (error - self.last_error)/dt

        self.integral = self.integral + error * dt

        output = self.kp * error + self.kd * derivative + self.ki * self.integral

        self.last_error = error

        return output

class Robot_PID(object):
    def __init__(self, robot, send_data=False):
        self.robot = robot
        self.game = self.robot.game

        self.desired = np.array([0, 0])
        self.linear_pid = PID(1, 0, 0)
        self.angular_pid = PID(100, 0, 0)
        self.power_left , self.power_right = 0, 0

        # self.pid_file = open("pid.log", "a")
        
    def update(self):
        linear_speed, angular_speed = self.robot._get_differential_robot_speeds(self.robot.vx, self.robot.vy, self.robot.theta)
        
        linear_speed, angular_speed = linear_speed * 100, angular_speed

        linear_desired, angular_desired = self.robot._get_desired_differential_robot_speeds(self.desired[0],self.desired[1], self.robot.theta)
        
        linear_desired, angular_desired =  linear_desired * 100, angular_desired
        linear_desired, angular_desired = 0, -1.5

        vl, va = self.update_Speed(linear_desired,angular_desired,linear_speed, angular_speed)
        
        acc_left  = vl - va
        acc_right = vl + va

        if self.game.vision._fps != 0:
            # self.power_left = self.power_left + acc_left * (1/self.game.vision._fps)
            # self.power_right = self.power_right + acc_right * (1/self.game.vision._fps)
            self.power_left = acc_left * (1/self.game.vision._fps)
            self.power_right = acc_right * (1/self.game.vision._fps)

            self.power_left = min(255, max(self.power_left, -255))
            self.power_right = min(255, max(self.power_right, -255))
            return self.power_left , self.power_right
        
        return 0, 0


    def set_desired(self, vector):
        self.desired = vector


    def update_Speed(self, linear_desired, angular_desired, now_linear, now_angular):
        self.linear_desired = linear_desired
        self.angular_desired = angular_desired

        self.linear_pid.set_desired_PID(linear_desired)
        self.angular_pid.set_desired_PID(angular_desired)

        vl, va = 0, 0
        if self.game.vision._fps != 0:
            vl = self.linear_pid.update_PID(now_linear, self.game.vision._fps)
            va = self.angular_pid.update_PID(now_angular, self.game.vision._fps)

        api.DataSender().get_node(f'PID_{self.robot.get_name()}').capture(
            desired_linear=self.linear_desired,
            now_linear=now_linear,
            new_linear=vl,
            desired_angular=self.angular_desired,
            now_angular=now_angular,
            new_angular=va,
        )
        # self.pid_file.write(str() + "," + str(now_linear) + "," + str(vl) + "," +  str(self.angular_desired) + "," + str(now_angular) + "," + str(0) + '\n')
        return vl, va
