import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector


class FollowBall(Strategy):
    def __init__(self, match):
        super().__init__(match)
        self.fields = algorithims.fields.PotentialField(self.match)


    def start(self, robot=None, speed=0.5, astar_timespan=0.1):
        super().start(robot=robot)

        def follow_ball(m):
            return (m.ball.x, m.ball.y)
        

        self.fields.add_field(
            algorithims.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = 1.8, # 30cm
                line_dist = 0.8,
                decay = None,
                multiplier = 0.8 # 80 cm/s
                )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return self.fields.compute([self.robot.x, self.robot.y])
