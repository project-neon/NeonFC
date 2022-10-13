from algorithms.limit_cycle import LimitCycle, Obstacle, Point
import math
from controller.PID_control import PID_control
from strategy.BaseStrategy import Strategy
import algorithms

class Attacker_LC(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "Limit_Cycle_Attacker", controller=PID_control)

        self.BALL_Y_MAX = 1.3
        self.BALL_Y_MIN = 0.2

        self.dl = 0.000001
        self.shooting_momentum = 0

    def can_shoot(self):
        ball = self.match.ball
        t_a = (ball.y - self.robot.y)/(ball.x - self.robot.x)
        proj = t_a*(1.5 - self.robot.x) + self.robot.y
        ball_behind = ball.x > self.robot.x

        return self.limit_cycle.target_is_ball and .45 < proj < .85 and ball_behind

    def start(self, robot=None):
        super().start(robot=robot)

        self.dl = 1 / self.match.game.vision._fps if self.match.game.vision._fps != 0 else self.dl
        self.controller.dl = self.dl

        self.limit_cycle = LimitCycle(self, target_is_ball=True)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def update_momentum(self):
        angle_between = lambda p1, p2 : math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        dist = lambda p1, p2: ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**.5

        theta = self.robot.theta
        x = self.robot.x
        y = self.robot.y
        ball = [self.match.ball.x, self.match.ball.y]
        goal = [1.5, .65]

        # diference between robot angle and (ball-robot) angle
        c1 = abs(theta - angle_between([x, y], ball)) <= .25

        # diference between robot angle and (goal-robot) angle
        c2 = abs(theta - angle_between([x, y], goal)) <= .5

        # distance between ball and robot
        c3 = dist([x, y], ball) <= .1

        if c1 and c2 and c3:
            self.shooting_momentum = 90 * dist([x, y], ball)

    def decide(self):
        x = self.robot.x
        y = self.robot.y

        ball_virtual_y = max(self.BALL_Y_MIN, min(self.BALL_Y_MAX, self.match.ball.y))

        robot = Point(x, y)
        target = Point(self.match.ball.x, ball_virtual_y)

        if not (0 <= target.x <= 1.5) and not (0 <= target.y <= 1.3):
            target = Point(self.limit_cycle.target.x, self.limit_cycle.target.y)

        boundaries = [Obstacle(x-.2, 0, r=.2), Obstacle(x-.2, 1.3, r=.2), Obstacle(0, y, r=.2), Obstacle(1.5, y, r=.2)]

        self.limit_cycle.update(robot, target, [*boundaries])
        desired = self.limit_cycle.compute()

        self.update_momentum()
        if self.shooting_momentum > 0:
            print("kicking ------------------ ")
            self.shooting_momentum -= 1
            desired = [1.5, .65]

        return desired
