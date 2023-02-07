import math
from collections import deque
from strategy.BaseStrategy import Strategy
from controller import PID_control, SimpleLQR, TwoSidesLQR, UniController
from algorithms.limit_cycle import LimitCycle

class LimitCycle_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, name='LimitCycle_Test', controller=PID_control)

        self.circuit = [(0.375, 0.25), (1.125, 0.25), (0.75, 0.65), (1.125, 1.05), (0.375, 1.05)]
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
        pass
