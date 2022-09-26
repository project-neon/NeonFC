from algorithms.limit_cycle import LimitCycle, Obstacle, Point
import math
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from collections import deque
from controller.uni_controller import UniController

class PID_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "PID_Test", controller=UniController)

        xs = [i/10 + .45 for i in range(10)]
        self.circuit = [(x, (1/3)*math.sin(5*math.pi*x/1.5)+0.65) for x in xs]
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

    def start(self, robot=None):
        super().start(robot=robot)

        target = Point(.75, .65)

        # for r in self.match.robots:
        #     if r.robot_id == 1:
        #         o1 = Obstacle(r.x, r.y, r=.2)

        self.dl = 1 / self.match.game.vision._fps if self.match.game.vision._fps != 0 else self.dl
        self.controller.dl = self.dl

        self.limit_cycle = LimitCycle(self, self.robot, [], self.match.ball, target_is_ball=True)

        if self.controller.__class__ is UniController:
            robot_dl = Point(self.robot.x + self.dl*math.cos(self.robot.theta),
                             self.robot.y + self.dl*math.sin(self.robot.theta))

            self.limit_cycle_dl = LimitCycle(self, robot_dl, [], self.match.ball, target_is_ball=True)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        # desired = self.next_point()
        desired = self.limit_cycle.compute()

        # print(self.robot.x, self.robot.y)
        # print(self.limit_cycle_dl.robot.x, self.limit_cycle_dl.robot.y)

        if self.controller.__class__ is UniController:
            self.limit_cycle_dl.update(
                self.robot.x + self.dl*math.cos(self.robot.theta),
                self.robot.y + self.dl*math.sin(self.robot.theta)
            )
            desired_dl = self.limit_cycle_dl.compute()

            return desired, desired_dl

        return desired
