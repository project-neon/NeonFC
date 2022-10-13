from algorithms.limit_cycle import LimitCycle, Obstacle, Point
import math
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from collections import deque
from controller.uni_controller import UniController
import algorithms
import time

class PID_Test(Strategy):
    def __init__(self, match, plot_field=False):
        self.lt = time.time()
        super().__init__(match, "PID_Test", controller=TwoSidesLQR)

        self.circuit = [(1.1, .40), (1.1, .90), (.4, .90), (.4, .40)]
        self.circuit = deque(self.circuit)
        self.dl = 0.000001

    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]

    def can_shoot(self):
        ball = self.match.ball
        t_a = (ball.y - self.robot.y)/(ball.x - self.robot.x)
        proj = t_a*(1.5 - self.robot.x) + self.robot.y
        ball_behind = ball.x > self.robot.x

        return self.limit_cycle.target_is_ball and .45 < proj < .85 and ball_behind

    def start(self, robot=None):
        super().start(robot=robot)

        # self.dl = 1 / self.match.game.vision._fps if self.match.game.vision._fps != 0 else self.dl
        # self.controller.dl = self.dl

        # self.limit_cycle = LimitCycle(self, target_is_ball=True)

        self.field = algorithms.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
        )

        self.field.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (.75, .65),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = .3
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        # robot = Point(self.robot.x, self.robot.y)
        # target = Point(.75, .65)

        # if not (0 <= target.x <= 1.5) and not (0 <= target.y <= 1.3):
        #     target = Point(self.limit_cycle.target.x, self.limit_cycle.target.y)

        # # for r in self.match.robots:
        # #     if r.robot_id == 1:
        # #         o1 = Obstacle(r.x, r.y, r=.2)

        # self.limit_cycle.update(robot, target, [])
        # desired = self.limit_cycle.compute()

        # if self.can_shoot():
        #     desired = target.x, target.y

        # # if self.controller.__class__ is UniController:

        # #     robot_dl = Point(
        # #         self.robot.x + self.dl*math.cos(self.robot.theta),
        # #         self.robot.y + self.dl*math.sin(self.robot.theta)
        # #     )

        # #     self.limit_cycle.update(robot_dl, target, [])
        # #     desired_dl = self.limit_cycle.compute()

        # #     if self.can_shoot():
        # #         desired = math.atan2(target.y - robot.y, target.x - robot.x)
        # #         desired_dl = math.atan2(target.y - robot_dl.y, target.x - robot_dl.x)

        # #     return desired, desired_dl

        # desired = self.next_point()

        # return desired

        behaviour = self.field

        return behaviour.compute([self.robot.x, self.robot.y])