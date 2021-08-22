from controller.simple_LQR import TwoSidesLQR
import math
import random

from scipy.spatial import Voronoi

from controller import TwoSidesLQR

from algorithms.astar.astar import AStar, Node
from algorithms.astar.fieldGraph import FieldGraph
from algorithms.potential_fields.fields import TangentialField, PotentialField, LineField
from strategy.BaseStrategy import Strategy

class DefensivePlay(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game

        self.graph = FieldGraph()

        super().__init__(match, 'DefensiveTest', controller=TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        self.defensive = PotentialField(self.match)


        self.defensive.add_field(
            LineField(
                self.match,
                theta=0,
                target= lambda m: [0, m.ball.y],
                line_size=0.25,
                line_dist=1,
                decay = lambda x: 1,
                multiplier = 0.75
            )
        )



        
    
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):


        return self.defensive.compute([self.robot.x, self.robot.y])
        
        

