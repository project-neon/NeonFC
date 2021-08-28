import algorithms
import math
import algorithms
import controller
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

def get_ball_info(m):
    return (m.ball.vx, m.ball.vy, m.ball.x, m.ball.y)

class newGoalKeeper(Strategy):
    def __init__(self, match, plot_field=True):
        super().__init__(match, "MktGoalKeeper", controller=controller.TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        self.restrict = algorithms.fields.PotentialField(self.match, name="RestrictBehaviour")

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")

        self.kalm = algorithms.fields.PotentialField(self.match, name="KalmBehaviour")

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

        self.path.add_field(self.restrict)
        
        def get_def_spot(m):
            x = w/2
            if m.ball.vy==0 or m.ball.vx==0:
                return (w/2, 1.3/2)
            y = (m.ball.vy/m.ball.vx)*(x-m.ball.x)+m.ball.y
            if y > (1.3/2)+0.2:
                y = (1.3/2)+0.2
            elif y < (1.3/2)-0.2:
                y = (1.3/2)-0.2
            return (x, y)

        self.path.add_field(
            algorithms.fields.LineField(
                self.match,
                target = get_def_spot,
                theta = 0,
                line_size = w/2,
                line_dist = 0.1,
                line_dist_max = 0.7,
                multiplier = 0.7,
                decay = lambda x : x
            )
        )

        # permanece no centro da área
        self.kalm.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (0.07, 0.65),
                theta = 0,
                line_size = w/2,
                line_dist = 0.1,
                line_dist_max = 0.7,
                decay = lambda x: x,
                multiplier = 0.7,
            )
        )

    def decide(self):

        behaviour = None

        if self.match.ball.x < 0.750 and self.match.ball.vx < 0:
            behaviour = self.path

        elif self.match.ball.x >= 0.750:
            behaviour = self.kalm

        else:
            behaviour = self.restrict

        #return super().decide(self.path)

        return behaviour.compute([self.robot.x, self.robot.y])
