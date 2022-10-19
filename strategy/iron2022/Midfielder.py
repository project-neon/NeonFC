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

        self.last_b_x = 0
        self.last_b_y = 0

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

        self.recovery = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|RecoveryBehaviour"
        )

        self.defender.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (.25, m.ball.y),
                theta = 0,
                line_size = 0.1,
                line_dist = 0.1,
                multiplier = 0.7,
                decay = lambda x : x**3
            )
        )

        self.push_ball.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.1,
                decay = lambda x: x,
                multiplier = .7
            )
        )

        self.recovery.add_field(
            algorithms.fields.PointField(
                self.match,
                target=(.35, .65),
                radius=0.25,
                decay=lambda x: x ** 2,
                multiplier=.7
            )
        )
        
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        behaviour = None

        ball = self.match.ball
        ball_x, ball_y = ball.x, ball.y

        if ball_x < .575 and ball_x > self.robot.x:
            behaviour = self.push_ball
        elif self.robot.x > .25 or self.robot.x < .15:
            behaviour = self.recovery
        else:
            behaviour = self.defender

        if ball_x == -1 or ball_y == -1:
            ball_x, ball_y = self.last_b_x, self.last_b_y

        # if ball_x < .375:
        #     self.pushing_ball = True

        # if self.pushing_ball:
        #     if ball_x >= .575 or ball_x < .15:
        #         self.pushing_ball = False
        #         behaviour = self.defender
        #     else:
        #         behaviour = self.push_ball
        # else:
        #     if self.robot.x > .25 or self.robot.x < .15:
        #         behaviour = self.recovery
        #     else:
        #         behaviour = self.defender

        if ball.x > 0 and ball.y > 0:
            self.last_b_x, self.last_b_y = ball.x, ball.y

        return behaviour.compute([self.robot.x, self.robot.y])

