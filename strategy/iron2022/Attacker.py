from strategy.BaseStrategy import Strategy
from controller.uni_controller import UniController
from algorithms import UnivectorField
import math


class Attacker(Strategy):
    def __init__(self, match, name="UVF_IRON"):
        super().__init__(match, name=name, controller=UniController)
        self.dl = 0.000001
        self.field = UnivectorField(n=6, rect_size=.0)
        self.shooting_momentum = 0
        self.USE_OBSTACLES = False

    def start(self, robot=None):
        super().start(robot=robot)

    def position_to_shoot(self, g, theta_d):
        # FIXME
        math.atan2(self.robot.x - 1.5, self.robot.y - .65)
        threshold_p = 0.3
        threshold_a = 0.6

        angle = abs(theta_d - self.robot.theta) < threshold_a
        position = ((g[0] - self.robot.x) ** 2 + (g[1] - self.robot.y) ** 2) ** .5 < threshold_p

        return angle and position

    def update_momentum(self):
        if self.position_to_shoot(self.field.g, self.desired_theta):
            distance_to_goal = ((self.robot.x-1.5) ** 2 + (self.robot.y-.65) ** 2) ** .5
            self.shooting_momentum = 90 * distance_to_goal

    def ball_in_field(self):
        return self.match.ball.x != -1 and self.match.ball.y != -1

    def default_behavior(self):
        x = self.robot.x
        y = self.robot.y

        R_DISPLACEMENT = .5
        g = (self.match.ball.x, self.match.ball.y)
        r = (g[0] - (g[0]-1.5) * R_DISPLACEMENT, g[1] - (g[1]-.65) * R_DISPLACEMENT)

        if self.ball_in_field():
            self.field.set_target(g, r)

        if self.USE_OBSTACLES:
            self.field.del_obstacle(all=True)

            for robot in self.match.robots:
                if (robot.x, robot.y) != (x, y):
                    self.field.add_obstacle(
                        (robot.x, robot.y),
                        0.075 * 1.4,
                        0.075 * 1.4
                    )

    def shoot(self):
        self.field.del_obstacle(all=True)
        g = (1.5, .65)
        self.field.set_target(g, g)

    def decide(self):
        self.dl = 1 / self.match.game.vision._fps if self.match.game.vision._fps != 0 else self.dl
        self.controller.dl = self.dl

        theta = self.robot.theta
        x = self.robot.x
        y = self.robot.y

        self.default_behavior()
        self.desired_theta = self.field((x, y))

        self.update_momentum()

        if self.shooting_momentum > 0:
            self.shoot()
            self.shooting_momentum -= 1

        theta_d = self.field((x, y))
        theta_f = self.field((x + self.dl*math.cos(theta), y + self.dl*math.sin(theta)))

        return theta_d, theta_f
