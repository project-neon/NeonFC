import algorithms
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "GoalkeeperIRON", controller=TwoSidesLQR)

        self.g_left = 0.87
        self.g_right = 0.46

    def start(self, robot=None):
        super().start(robot=robot)

        self.pebas = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|PebasBehaviour"
        )

        self.left_edge = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|LeftEdgeBehaviour"
        )

        self.right_edge = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|RightEdgeBehaviour"
        )

        self.recovery = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|RecoveryBehaviour"
        )

        self.push_ball = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|PushBallBehaviour"
        )

        self.pebas.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (0.075, m.ball.y),
                theta = 0,
                line_size = 0.1,
                line_dist = 0.1,
                multiplier = 0.5,
                decay = lambda x : x**2
            )
        )

        self.left_edge.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.075, self.g_left),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = .3
            )
        )

        self.right_edge.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.075, self.g_right),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = .3
            )
        )

        self.push_ball.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (0.075, m.ball.y),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = .3
            )
        )

        self.recovery.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.075, .65),
                radius = 0.25,
                decay = lambda x: x**2,
                multiplier = .3
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):

        behaviour = None

        ball = self.match.ball

        if not self.g_right < ball.y < self.g_left and ball.x < .15:
            behaviour = self.push_ball
        elif ball.y > self.g_left:
            behaviour = self.left_edge
        elif ball.y < self.g_right:
            behaviour = self.right_edge
        else:
            behaviour = self.pebas

        if self.robot.x > 0.075:
            behaviour = self.recovery

        # print(self.robot.y)

        return behaviour.compute([self.robot.x, self.robot.y])

    def update(self):
        if self.robot.x <= .075 and (not -1.61 < self.robot.theta < -1.57 or not 1.57 < self.robot.theta < 1.61):
            w = abs(self.robot.theta) - 1.57
            return 0, w
        return self.controller.update()