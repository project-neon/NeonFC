import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector


class GoalKeeper(Strategy):
    def __init__(self, match):
        super().__init__(match)
        self.fields = algorithims.fields.PotentialField(self.match)


    def start(self, robot=None):
        super().start(robot=robot)

        def follow_ball(m):
            return (m.ball.x, m.ball.y)
        
        def quadratic(x):
            return x**2
        

        self.fields.add_field(
            algorithims.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = 1.8, # 30cm
                line_dist = 0.8,
                decay = None,
                multiplier = 0.5 # 50 cm/s
                )
        )

        self.fields.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0 + 0.4, 0),
                theta = math.pi/2,
                line_size = 1.8, # 30cm
                line_dist = 0.8,
                decay = None,
                multiplier = 0.5 # 50 cm/s
                )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return self.fields.compute([self.robot.x, self.robot.y])
