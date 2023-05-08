from strategy.BaseStrategy import Strategy
from controller.uni_controller import UniController
from controller import PID_control
from algorithms.univector_field import UnivectorField
import math


class Attacker(Strategy):
    def __init__(self, match, name="UVF_Test_2"):
        super().__init__(match, name=name, controller=PID_control)
        self.dl = 0.000001
        self.field = UnivectorField(n=6, rect_size=.2)
        self.shooting_momentum = 0

    def start(self, robot=None):
        super().start(robot=robot)

    def position_to_shoot(self, g, theta_d):
        math.atan2(self.robot.x - 1.5, self.robot.y - .65)
        threshold_p = 0.3
        threshold_a = 0.6

        angle = abs(theta_d - self.robot.theta) < threshold_a
        print(angle)
        position = ((g[0] - self.robot.x) ** 2 + (g[1] - self.robot.y) ** 2) ** .5 < threshold_p
        print(position)

        return angle and position

    def decide(self):
        self.dl = 1 / self.match.game.vision._fps if self.match.game.vision._fps != 0 else self.dl
        self.controller.dl = self.dl

        theta = self.robot.theta
        x = self.robot.x
        y = self.robot.y

        g = (self.match.ball.x, self.match.ball.y)  # (.75, .65)
        delta_x = g[0] - (g[0] - 1.5) * .05
        delta_y = g[1] - (g[1] - .65) * .05
        r = (delta_x, delta_y)

        if 0 <= g[0] <= 1.5 and 0 <= g[1] <= 1.3:
            self.field.set_target(g, r)

        self.field.del_obstacle(all=True)
        # for robot in self.match.robots:
        #     if (robot.x, robot.y) != (x, y) and (robot.x, robot.y) != (0, 0):
        #         self.field.add_obstacle(
        #             (robot.x, robot.y),
        #             0.075 * 1.4,
        #             0.075 * 1.4
        #         )

        theta_d = self.field((x, y))

        g = g if 0 <= g[0] <= 1.5 and 0 <= g[1] <= 1.3 else self.field.g
        if self.position_to_shoot(g, theta_d):
            distance_to_goal = ((self.robot.x - 1.5) ** 2 + (self.robot.y - .65) ** 2) ** .5
            self.shooting_momentum = 90 * distance_to_goal

        if self.shooting_momentum > 0:
            g = (1.5, .65)
            self.field.set_target(g, g)
            theta_d = self.field((x, y))
            self.shooting_momentum -= 1

        desired = [math.cos(theta_d)+x, math.sin(theta_d)+y]
        print("saaa", desired)
        return desired
