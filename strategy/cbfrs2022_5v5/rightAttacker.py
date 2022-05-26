from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
from controller.simple_LQR import TwoSidesLQR
import algorithms


# class RightAttacker(DebugPotentialFieldStrategy):
class RightAttacker(Strategy):
    def __init__(self, match, name="RightAttacker"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def start(self, robot=None):
        super().start(robot=robot)

        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|PointBehaviour".format(self.__class__)
        )

        self.push = algorithms.fields.PotentialField(
            self.match,
            name="{}|PushBehaviour".format(self.__class__)
        )

        self.attack = algorithms.fields.PotentialField(
            self.match,
            name="{}|AttackBehaviour".format(self.__class__)
        )

        self.change_position = algorithms.fields.PotentialField(
            self.match,
            name="{}|ChangePositionBehaviour".format(self.__class__)
        )

        self.defend.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.sa_x + 0.4, self.field_h/2),
                radius = .075,
                decay = lambda x: x**6,
                multiplier = 1
            )
        )

        self.push.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x - 0.3, m.ball.y - 0.05),
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

        self.attack.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x - 0.3, (self.field_h/2 - 0.15)),
                radius = .075,
                decay = lambda x: x**6,
                multiplier = 1
            )
        )

        self.change_position.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.field_w/2, 0.15),
                radius = .075,
                decay = lambda x: x**6,
                multiplier = 1
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        ball = self.match.ball

        behaviour = self.defend

        # if ball.x > self.sa_w + 0.3 and self.robot.x > ball.x:
        #     behaviour = self.change_position
        # else:
        if ball.y <= self.sa_y + self.sa_h + 0.1 and ball.y >= self.sa_y - 0.1 and ball.x > self.sa_w + 0.3:
            behaviour = self.push
        else:
            if ball.x > self.field_w/4:
                behaviour = self.attack
            else:
                behaviour = self.defend

        return behaviour.compute([self.robot.x, self.robot.y])