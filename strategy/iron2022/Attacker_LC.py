from algorithms.limit_cycle import LimitCycle, Obstacle, Point
import math
from controller.PID_control import PID_control
from strategy.BaseStrategy import Strategy
import algorithms

class Attacker_LC(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "Limit_Cycle_Attacker", controller=PID_control)

        self.BALL_Y_MAX = 1.2
        self.BALL_Y_MIN = 0.3

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
        c1v = abs(theta - angle_between([x, y], ball))
        c1 = c1v <= .4 or abs(c1v - math.pi) <= .4

        # diference between robot angle and (goal-robot) angle
        c2v = abs(theta - angle_between([x, y], goal)) #<= .5
        c2 = c2v <= .4 or abs(c2v - math.pi) <= .4

        # distance between ball and robot
        c3 = dist([x, y], ball) <= .15

        # check if all is between goal and robot
        c4 = ball[0] > self.robot.x

        if c3:
            print(f"{c1v=}, {c2v=}")
            print(c1, c2, c3)

        if c1 and c2 and c3 and c4:
            self.shooting_momentum = 120 * dist([x, y], goal)

        elif ball != [-1, -1]:
            self.shooting_momentum -= 30

    def decide(self):
        x = self.robot.x
        y = self.robot.y
        print('posssss', x, y)

        ball_virtual_y = max(self.BALL_Y_MIN, min(self.BALL_Y_MAX, self.match.ball.y))

        robot = Point(x, y)
        target = Point(self.match.ball.x, self.match.ball.y)

        if not (0 <= target.x <= 1.5) and not (0 <= target.y <= 1.3):
            target = Point(self.limit_cycle.target.x, self.limit_cycle.target.y)

        #boundaries = [Obstacle(x-.2, 0, r=.2), Obstacle(x-.2, 1.3, r=.2), Obstacle(0, y, r=.2), Obstacle(1.5, y, r=.2)]

        self.limit_cycle.update(robot, target, [])#, [*boundaries])
        desired = self.limit_cycle.compute()

        self.update_momentum()
        if self.shooting_momentum > 0:
            print("kicking ------------------")
            self.controller.K_RHO = -10
            self.controller.lp = [1.5, .65]
            self.controller.control_linear_speed = True
            desired = [1.5, .65]
            self.shooting_momentum -= 1
        else:
            self.controller.K_RHO = 0.05
            self.controller.control_linear_speed = True
            self.controller.lp = [self.match.ball.x, self.match.ball.y]

        bound_r = .15

        if self.match.ball.y >= self.BALL_Y_MAX:
            v_o = Obstacle(self.match.ball.x, self.BALL_Y_MAX + bound_r, r=bound_r)
            self.limit_cycle.update(robot, target, [v_o], target_is_ball=False)
            desired = self.limit_cycle.compute()

        if self.match.ball.y <= self.BALL_Y_MIN:
            v_o = Obstacle(self.match.ball.x, self.BALL_Y_MIN - bound_r, r=bound_r)
            self.limit_cycle.update(robot, target, [v_o], target_is_ball=False)
            desired = self.limit_cycle.compute()

        if self.match.ball.x <= .375:
            if self.match.ball.y < .65:
                desired = [.5, 1.05]
            else:
                desired = [.5, .25]


        return desired
