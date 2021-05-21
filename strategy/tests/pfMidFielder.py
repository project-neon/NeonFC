import math
import algorithms
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class MidFielder(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match)

        """
        Ainda precisa ser feito
        """
        
        self.plot_field = plot_field
        self.exporter = None

        self.base_rules = algorithms.fields.PotentialField(
            self.match,
            name="{}|BaseRulesBehaviour".format(self.__class__)
        )



    def start(self, robot=None):
        super().start(robot=robot)

        if self.plot_field:
            self.exporter = algorithms.fields.PotentialDataExporter(self.robot.get_name())


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        dist_to_ball = np.linalg.norm(
            np.array([self.robot.x, self.robot.y]) - 
            np.array([self.match.ball.x, self.match.ball.y])
        )

        behaviour = self.base_rules

        return behaviour.compute([self.robot.x, self.robot.y])

