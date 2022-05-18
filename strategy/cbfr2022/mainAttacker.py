from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
import math
import algorithms

# class MainAttacker(DebugPotentialFieldStrategy):
class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=UniController)

    def start(self, robot=None):
        super().start(robot=robot)

        self.point = algorithms.fields.PotentialField(
            self.match,
            name="{}|PointBehaviour".format(self.__class__)
        )

        self.point.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.75, 0.65),
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self, x=None, y=None):
        if x:
            self.robot.x = x
        if y:
            self.robot.y = y

        behaviour = None

        behaviour = self.point

        return behaviour.compute([self.robot.x, self.robot.y])
        
        
