import math
import random

import numpy as np

from algorithms.dijkstra_waypoint.waypoint import WaypointSystem
from algorithms.dijkstra_waypoint import dijkstra

from algorithms.potential_fields.fields import PotentialField, TangentialField

from strategy.BaseStrategy import Strategy

from commons.math import dist_point_line

class DummyEntity:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.name = "DUMMY_" + str(int(random.random() * 40000))

    def get_name(self): return self.name

class DefensivePlay(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game

        self.decider = WaypointSystem()

        field_h = 1.30
        field_gap = 0.04 # 4 cm
        self.common_vertex = [
            # DummyEntity(field_gap, field_gap),
            # DummyEntity(field_gap, field_h - field_gap),
            # DummyEntity(field_h - field_gap, field_gap),
            # DummyEntity(field_h - field_gap, field_h - field_gap)
        ]
        super().__init__(match, 'OffensiveWaypointStrategy')

    def start(self, robot=None):
        super().start(robot=robot)
        self.tangential = None
    
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):
        ball = self.match.ball
        robot = self.robot
        self.decider = WaypointSystem()
        obstacles = [a for a in self.match.robots if a.robot_id != self.robot.robot_id] + self.match.opposites

        dist_to_ball = ( (ball.x - robot.x)**2 +  (ball.y - robot.y)**2 )**.5
        if dist_to_ball >= 0.3:
            self.tangential = None
            self.decider.update(self.robot, ball, obstacles + self.common_vertex)

            return self.decider.decide(self.robot, ball, 0.8)
        else:
            if self.tangential:
                return self.tangential.compute([self.robot.x, self.robot.y])
            else:
                self.tangential = TangentialField(
                    self.match,
                    target=lambda m: (
                        m.ball.x + (math.cos(math.pi/3) if m.ball.y < 0.65 else math.cos(5*math.pi/3)) * 0.4 * dist_to_ball,
                        m.ball.y + (math.sin(math.pi/3) if m.ball.y < 0.65 else math.sin(5*math.pi/3)) * 0.4 * dist_to_ball
                    ),                                                                                                                                                                                                                                                                                                                                          
                    radius = dist_to_ball * 0.2,
                    radius_max = dist_to_ball * 10,
                    clockwise = lambda m: (m.ball.y < 0.65),
                    decay=lambda x: 1,
                    field_limits = [0.75* 2 , 0.65*2],
                    multiplier = 1
                )

                return self.tangential.compute([self.robot.x, self.robot.y])
        
        

