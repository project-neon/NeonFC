import math
import algorithms
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class DwaScratch(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game
        super().__init__(match)

    def start(self, robot=None):
        self.dwa = algorithms.dwa.DynamicWindowApproach(robot, self.game)
        self.controller = self.dwa
        if (robot):
            self.robot = robot

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot

    def set_desired(self, desired):
        return 

    def decide(self):
        return 0, 0

