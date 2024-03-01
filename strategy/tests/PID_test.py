from NeonPathPlanning import LimitCycle
import math
from controller.PID_control import PID_control, PID_W_control
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from collections import deque
from controller.uni_controller import UniController
import algorithms
import time

class PID_Test(Strategy):
    def __init__(self, match, plot_field=False):
        self.lt = time.time()
        super().__init__(match, "PID_Test", controller=PID_W_control)

        self.circuit = [(1.1, .40), (1.1, .90), (.35, .65)]#, (.4, .90), (.4, .40)]
        self.circuit = deque(self.circuit)
        self.dl = 0.000001

    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):

        desired = self.next_point()

        return desired
