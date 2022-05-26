from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
from controller.simple_LQR import TwoSidesLQR
import algorithms

# class RightWing(DebugPotentialFieldStrategy):
class RightWing(Strategy):
    def __init__(self, match, name="RightWing"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def start(self, robot=None):
        super().start(robot=robot)

        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|DefendBehaviour".format(self.__class__)
        )

        self.combat = algorithms.fields.PotentialField(
            self.match,
            name="{}|CombatBehaviour".format(self.__class__)
        )

        self.attack = algorithms.fields.PotentialField(
            self.match,
            name="{}|AttackBehaviour".format(self.__class__)
        )

        self.defend.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, (self.sa_y - 0.2)),
                radius = .075,
                decay = lambda x: x**6,
                multiplier = 1
            )
        )

        self.combat.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x - 0.4, m.ball.y),
                radius = 2.2,
                decay = lambda x: 1,
                multiplier = 1
            )
        )

        self.attack.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x - 0.4, (self.sa_y - 0.2)),
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

        if ball.y < self.sa_y - 0.1 and ball.x > self.robot.x:
            behaviour = self.combat
        else:
            if ball.x > self.sa_w + 0.2 and ball.y > self.sa_y - 0.15:
                behaviour = self.attack
            else:
                behaviour = self.defend

        return behaviour.compute([self.robot.x, self.robot.y])