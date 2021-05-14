import math
import algorithms
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector
from controller.PID import Robot_PID

import json
import numpy as np

class ControlSystemTroubleshoot(Strategy):
    def __init__(self, match):
        super().__init__(match,
            'PIDTest',
            controller=Robot_PID
        )


    def start(self, robot=None):
        super().start(robot=robot)

        self.test = algorithms.fields.PotentialField(
            self.match, 
            name="{}|TestBehaviour".format(self.__class__)
        )

        self.test.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y), # segue a bola
                radius = 0.1, # 30cm
                decay = lambda x: 1,
                multiplier = 0.5 # 50 cm/s
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return self.test.compute([self.robot.x, self.robot.y])

