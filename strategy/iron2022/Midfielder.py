import algorithms
import math
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy

class Midfielder(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "FullBeck", controller=TwoSidesLQR)

        self.wing_x = 0.66
        self.lwr_margin = 0.2
        self.hgr_margin = 1.1

        self.def_x = 0.27
        self.ga_hgr = 1
        self.ga_lwr = .3

        self.pushing_ball = False

    def start(self, robot=None):
        super().start(robot=robot)

        self.defender = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|DefenderBehaviour"
        )

        self.push_ball = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|PushBallBehaviour"
        )

        self.defender.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (.25, m.ball.y),
                theta = 0,
                line_size = 0.1,
                line_dist = 0.1,
                multiplier = 0.5,
                decay = lambda x : x**3
            )
        )

        self.push_ball.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.1,
                decay = lambda x: x,
                multiplier = .5
            )
        )
        
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        behaviour = None

        ball = self.match.ball

        if ball.x < .375:
            self.pushing_ball = True

        if self.pushing_ball:
            if self.robot.x >= .575:
                self.pushing_ball = False
                behaviour = self.defender
            else:
                behaviour = self.push_ball
        else:
            behaviour = self.defender

        
        return behaviour.compute([self.robot.x, self.robot.y])

