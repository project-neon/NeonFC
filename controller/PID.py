import math

class PID(object):
    def __init__(self, robot, kp, kd ,ki):
        self.desired_PID = 0.0
        
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.last_error = 0

    def set_desired_PID(self, speed):
        self.desired_PID = speed

    def update_PID(self, now, fps):
        self.now = now
        dt = 1/fps

        error = self.desired_PID - now

        derivative = (error - self.last_error)/dt

        integral = integral + error * dt

        output = self.kp * error + self.kd * derivative + self.ki * integral

        self.last_error = error

        return output

   

class Robot_PID(object):
