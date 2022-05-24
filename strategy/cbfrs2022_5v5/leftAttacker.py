from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
from controller.simple_LQR import TwoSidesLQR
import algorithms
import math

# class LeftAttacker(DebugPotentialFieldStrategy):
class LeftAttacker(Strategy):
    def __init__(self, match, coach, name="LeftAttacker"):
        super().__init__(match, name, controller=TwoSidesLQR)
        self.coach = coach

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def start(self, robot=None):
        super().start(robot=robot)

        self.point = algorithms.fields.PotentialField(
            self.match,
            name="{}|PointBehaviour".format(self.__class__)
        )

        self.wait = algorithms.fields.PotentialField(
            self.match,
            name="{}|WaitBehaviour".format(self.__class__)
        )

        self.point.add_field(
            algorithms.fields.PointField(
                self.match,
                target = ((3/4)*self.field_w, (3/4)*self.field_h),
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

        self.wait.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.field_w, (3/4)*self.field_h),
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
        r_b_dist = self.coach.ball_dists.get(f"{self.robot.robot_id}")
        for id in self.coach.ball_dists:
            if self.coach.ball_dists[id] < r_b_dist:
                behaviour = self.wait
                break
            else:
                behaviour = self.point

        return behaviour.compute([self.robot.x, self.robot.y])