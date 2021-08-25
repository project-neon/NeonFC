import algorithms
import math
import algorithms
import controller
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

class mktGoalKeeper(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(
            match, "MktGoalKeeper", controller=controller.TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        self.restrict = algorithms.fields.PotentialField(self.match, name="RestrictBehaviour")

        x, y, w, h = self.match.game.field.get_small_area("defensive")

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (w, y+h/2),
                theta = math.pi/2,
                line_size = 1.3,
                line_dist = 0.15,
                line_dist_max = 0.3,
                inverse = True,
                line_dist_single_side = True,
                multiplier = 0.5,
                decay = lambda x : x
            )
        )

        self.restrict.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0, 0),
                radius = .12,
                radius_max = .12,
                decay = lambda x : 1,
                multiplier = -.75
            )
        )


    def decide(self):

        behaviour = None

        behaviour = self.restrict

        return behaviour.compute([self.robot.x, self.robot.y])

