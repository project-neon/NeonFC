import algorithms
import math
import algorithms
import controller
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

'''
esquerda: theta = 0
direita: theta = math.pi
'''

class mktGoalKeeper(Strategy):
    def __init__(self, match, plot_field=True):
        super().__init__(match, "MktGoalKeeper", controller=controller.TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        print(robot.robot_id)

        self.restrict = algorithms.fields.PotentialField(self.match, name="RestrictBehaviour")

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        x, y, w, h = self.match.game.field.get_small_area("defensive")

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (w, y+h/2),
                theta = math.pi/2,
                line_size = 1.3/2,
                line_dist = 0.15,
                line_dist_max = 0.3,
                line_dist_single_side = True,
                inverse = True,
                multiplier = 0.75,
                decay = lambda x : x
            )
        )

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, y+h/2),
                theta = 3*math.pi/2,
                line_size = h/2,
                line_dist = 0.075,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                inverse = True,
                multiplier = 0.75,
                decay = lambda x : x
            )
        )

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 0.25),
                theta = math.pi,
                line_size = 1.3,
                line_dist = 0.1,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                inverse = True,
                multiplier = 0.75,
                decay = lambda x : x
            )
        )

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 1.3-0.25),
                theta = 0,
                line_size = 1.3,
                line_dist = 0.1,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                inverse = True,
                multiplier = 0.75,
                decay = lambda x : x
            )
        )

        self.project.add_field(self.restrict)

        self.project.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m : (w/2, m.ball.y),
                radius = 0.05,
                decay = lambda x : 1,
                multiplier = 0.5
            )
        )

    def decide(self):

        behaviour = None

        if self.match.ball.x <= 0.750:
            behaviour = self.project
        else:
            behaviour = self.restrict

        #return super().decide(self.project)

        return behaviour.compute([self.robot.x, self.robot.y])
