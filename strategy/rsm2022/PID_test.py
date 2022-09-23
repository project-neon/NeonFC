from algorithms.limit_cycle import LimitCycle, Obstacle
import math
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from collections import deque
import json
import numpy as np

class PID_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "PID_Test", controller=PID_control)

        xs = [i/10 + .45 for i in range(10)]
        self.circuit = [(x, (1/3)*math.sin(5*math.pi*x/1.5)+0.65) for x in xs]
        self.circuit = deque(self.circuit)

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

        # for r in self.match.robots:
        #     if r.robot_id == 9 or r.robot_id == 0:
        #         o1 = Obstacle(r.x, r.y, r=.2)

        self.limit_cycle = LimitCycle(self.robot, [], self.match.ball, target_is_ball=True)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        desired = self.next_point() # self.limit_cycle.compute()

        return desired
