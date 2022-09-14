import algorithms
import math
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
import json
import numpy as np

class PID_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "PID_Test", controller=PID_control)

        self.circuit = [(.75, .65)]

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        
        return self.circuit[0]