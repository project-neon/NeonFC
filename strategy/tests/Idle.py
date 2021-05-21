import math
import algorithms
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class Idle(Strategy):
    def __init__(self, match):
        super().__init__(match, 'Idle')


    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return 0, 0

