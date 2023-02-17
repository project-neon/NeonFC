from algorithms.limit_cycle import LimitCycle
import math
from controller.PID_control import PID_W_control
from strategy.BaseStrategy import Strategy
from commons.math import distance_between_points


class Attacker_LC(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "Main_Attacker", controller=PID_W_control)

    def start(self, robot=None):
        super().start(robot=robot)
        self.limit_cycle = LimitCycle(self.match)
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        ball = (self.match.ball.x, self.match.ball.y)
        robot = (self.robot.x, self.robot.y)

        if distance_between_points(ball, robot) < 0.1 and not self.behind_ball():
            desired = (1.5, 0.65)
        else:
            self.limit_cycle.set_target(ball)
            self.limit_cycle.add_obstacle(self.get_virtual_obstacle(ball))
            desired = self.limit_cycle.compute(self.robot, fitness=20)

        return desired

    def get_virtual_obstacle(self, target):
        '''
        - m:    angle of the line perpendicular to the line between the ball and
                the center of the goal
        - p:    distance of the virtual obstacles to the ball / radius of the virtual obstacle
        - vo:   virtual obstacle
        - j:    this is the angle between the ball and the center of the goal
        - m:    the normal angle perpendicular to j
        - r:    radius of the ball
        '''
        aim_point = [self.field_w, self.field_h / 2]

        j = math.atan2(aim_point[1] - target[1], aim_point[0] - target[0])
        m = j + math.pi / 2
        p = 0.1

        r = .0427 / 2

        '''
        the terms r*cos(j) and r*sin(j) are subtracted to move
        the center of the obstacles behind the ball instead of its center
        '''
        if self.robot.y < math.tan(j) * (self.robot.x - target[0]) + target[1]:
            virtual_obstacle = (
            target[0] - p * math.cos(m) - r * math.cos(j), target[1] - p * math.sin(m) - r * math.sin(j), p, 1)
        else:
            virtual_obstacle = (
            target[0] + p * math.cos(m) - r * math.cos(j), target[1] + p * math.sin(m) - r * math.sin(j), p, -1)

        return virtual_obstacle
