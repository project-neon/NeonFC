from algorithms.univector_field import UnivectorField
import math
from controller.uni_controller import UniController
from strategy.BaseStrategy import Strategy
from commons.math import distance_between_points

class ShadowAttacker(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "Shadow_Attacker", controller=UniController, controller_kwargs={'control_speed': True})

    def start(self, robot=None):
        super().start(robot=robot)
        self.univector_field = UnivectorField(n=2, rect_size=0)
        self.dl = 0.000001

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        ball = (self.match.ball.x, self.match.ball.y)
        main_st = [[i.x, i.y] for i in self.match.robots if i.strategy.name == "Main_Attacker"][0]
        obs_radius = distance_between_points(main_st, ball)
        target = main_st[:]

        # second attacker offset on x based on the distance of the main attacker to the ball
        target[0] -= max(4*0.075, obs_radius)
        # second attacker offset on y based on the distance of the ball to the center
        target[1] += .5*(.65-ball[1])

        self.univector_field.set_target(target, ball)
        self.univector_field.del_obstacle(all=True)
        self.univector_field.add_obstacle(main_st, 0.075*1.4, obs_radius-0.075*1.4)

        x, y = self.robot.x, self.robot.y
        theta_d = self.univector_field.compute((x, y))
        theta_f = self.univector_field.compute((x + self.dl * math.cos(self.robot.theta), y + self.dl * math.sin(self.robot.theta)))

        self.controller.target = target

        return theta_d, theta_f
