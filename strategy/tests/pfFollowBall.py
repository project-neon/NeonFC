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
            algorithims.fields.PointField(
                self.match,
                target = follow_ball,
                radius = 0.3, # 30cm
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
