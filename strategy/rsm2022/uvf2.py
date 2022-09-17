from strategy.BaseStrategy import Strategy
from controller.uni_controller import UniController
from algorithms import UnivectorField
import math


class UVFAgent2(Strategy):
    def __init__(self, match, name="UVF_Test_2"):
        super().__init__(match, name=name, controller=UniController)
        self.dl = 0.000001
        self.field = UnivectorField(n=8, rect_size=.06, plot=True, path="uvf_plot.json")
        self.vs=[1]
        self.ws=[1]

    def start(self, robot=None):
        super().start(robot=robot)

    def decide(self):
        self.dl = 1 / self.match.game.vision._fps if self.match.game.vision._fps != 0 else self.dl
        self.controller.dl = self.dl

        theta = self.robot.theta
        x = self.robot.x
        y = self.robot.y

        g = (self.match.ball.x, self.match.ball.y)  # (.75, .65)
        delta_x = g[0] - (g[0]-1.5)*.05
        delta_y = g[1] - (g[1]-.65)*.05
        r = (delta_x, delta_y)

        if 0 <= g[0] <= 1.5 and 0 <= g[1] <= 1.3:
            self.field.set_target(g, r)

        self.field.del_obstacle(all=True)
        for robot in self.match.robots:
            if (robot.x, robot.y) != (x, y) and (robot.x, robot.y) != (0, 0):
                self.field.add_obstacle(
                    (robot.x, robot.y),
                    0.075*1.4*1,
                    0.075*1.4*1
                )
        for robot in self.match.opposites:
            if (robot.x, robot.y) != (0, 0):
                self.field.add_obstacle(
                    (robot.x, robot.y),
                    0.075*1.4*0.5,
                    0.075*1.4*0.3
                )
        self.field.save()

        theta_d = self.field((x, y))
        theta_f = self.field((x + self.dl*math.cos(theta), y + self.dl*math.sin(theta)))

        print(f"theta: {theta:.2f}, error: {theta_d-theta:.2f}, phi: {(theta_f-theta_d)/self.dl:.2f}")

        if len(self.vs) < 150:
            self.vs.append(self.robot.speed)
            self.ws.append(self.robot.vtheta)
        print(f"(x, y): ({x}, {y}), (w, v, rw): ({sum(self.ws)/len(self.ws)}, {sum(self.vs)/len(self.vs)}, {300/(sum(self.ws)/len(self.ws))})")
        return theta_d, theta_f
