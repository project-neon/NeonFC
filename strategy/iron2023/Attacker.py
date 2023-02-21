import numpy as np
from algorithms.limit_cycle import LimitCycle
import math
from controller.PID_control import PID_W_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay
from commons.math import distance_between_points


class MainPlay(PlayerPlay):
    def __init__(self, match, robot):
        # super().__init__(match, "Main_Attacker", controller=PID_W_control)
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Main Attacker Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        self.robot.strategy.controller = controller(self.robot)

    def start(self):
        self.limit_cycle = LimitCycle(self.match)
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def update(self):
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
                target[0] - p * math.cos(m) - r * math.cos(j),
                target[1] - p * math.sin(m) - r * math.sin(j),
                p,
                1
            )
        else:
            virtual_obstacle = (
                target[0] + p * math.cos(m) - r * math.cos(j),
                target[1] + p * math.sin(m) - r * math.sin(j),
                p,
                -1
            )

        if self.behind_ball():
            virtual_obstacle = (
                target[0] - p * math.cos(m) - r * math.cos(j),
                target[1] - p * math.sin(m) - r * math.sin(j),
                2 * p,
                np.sign(target[1] - p * math.sin(m) - r * math.sin(j) - 0.65)
            )

        return virtual_obstacle

    def behind_ball(self):
        # Convert input to numpy arrays for easy calculation
        point_on_line = np.array((self.match.ball.x, self.match.ball.y))
        point_on_normal = np.array((1.5, .65))
        point_to_check = np.array((self.robot.x, self.robot.y))

        # Calculate the normal vector of the line
        normal = point_on_normal - point_on_line

        # Calculate the vector from the point on the line to the point to check
        vector_to_check = point_to_check - point_on_line

        # Calculate the dot product of the normal vector and the vector to check
        dot_product = np.dot(normal, vector_to_check)

        # Check the sign of the dot product to determine if the point is to the right or left of the line
        return dot_product > 0

