import math
import algorithims
import controller
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector, distance

import json
import numpy as np

def point_in_rect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

class Avoid(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "avoid", controller=controller.TwoSidesLQR)

        """
        """
        
        self.normal_speed = 0.75
        self.obey_rules_speed = 0.5
        self.push_speed = 0.8

        self.plot_field = plot_field
        self.exporter = None

    def start(self, robot=None):
        super().start(robot=robot)

        
        if self.plot_field:
            self.exporter = algorithims.fields.PotentialDataExporter(self.robot.get_name())

        self.avoid = algorithims.fields.PotentialField(
            self.match,
            name="{}|AvoidBehaviour".format(self.__class__)
        )


        if self.robot.robot_id != 0:
            self.avoid.add_field(
                algorithims.fields.PointField(
                    self.match,
                    target= lambda m: (m.robots[0].x, m.robots[0].y),
                    radius=0.35,
                    radius_max=0.35,
                    decay = lambda x: x**5 - 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1.5
                )
            )

        if self.robot.robot_id != 1:
            self.avoid.add_field(
                algorithims.fields.PointField(
                    self.match,
                    target= lambda m: (m.robots[1].x, m.robots[1].y),
                    radius=0.35,
                    radius_max=0.35,
                    decay = lambda x: x**5 - 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1.5
                )
            )

        if self.robot.robot_id != 2:
            self.avoid.add_field(
                algorithims.fields.PointField(
                    self.match,
                    target= lambda m: (m.robots[2].x, m.robots[2].y),
                    radius=0.35,
                    radius_max=0.35,
                    decay = lambda x: x**5 - 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1.5
                )
            )



    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return self.avoid.compute([self.robot.x, self.robot.y])

