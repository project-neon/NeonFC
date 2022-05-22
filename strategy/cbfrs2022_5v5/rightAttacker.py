from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
from controller.simple_LQR import TwoSidesLQR
import algorithms

# class RightAttacker(DebugPotentialFieldStrategy):
class RightAttacker(Strategy):
    def __init__(self, match, name="RightAttacker"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def start(self, robot=None):
        super().start(robot=robot)

        self.point = algorithms.fields.PotentialField(
            self.match,
            name="{}|PointBehaviour".format(self.__class__)
        )

        self.point.add_field(
            algorithms.fields.PointField(
                self.match,
                target = ((3/4)*self.field_w, (1/4)*self.field_h),
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        behaviour = self.point

        return behaviour.compute([self.robot.x, self.robot.y])