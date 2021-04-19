import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class DijkstraSeeker(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game
        super().__init__(match, 'Idle')


    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        dijkstra = algorithims.dijkstra_waypoint.WaypointSystem()

        obstacles = [a for a in self.match.robots if a.robot_id != self.robot.robot_id] + self.match.opposites
        
        dijkstra.update(self.robot, self.match.ball, obstacles)
        
        # print(dijkstra.decide(self.robot.get_name(), self.match.ball.get_name()))
        return dijkstra.decide(self.robot, self.match.ball)

