import math

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

    def __init__(self, linear_pid, angular_pid):
        self.linear_pid = linear_pid
        self.angular_pid = angular_pid

    def update_Speed(self, linear_desired, angular_desired, now_linear, now_angular, fps):
        self.linear_desired = linear_desired
        self.angular_desired = angular_desired

        self.linear_pid.set_desired_PID(linear_desired)
        self.angular_pid.set_desired_PID(angular_desired)

        self.linear_pid.update_PID(now_linear,fps)
        self.angular_pid.update_PID(now_angular,fps)

