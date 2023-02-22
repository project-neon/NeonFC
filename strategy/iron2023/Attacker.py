import numpy as np
from algorithms.limit_cycle import LimitCycle
import math
from controller.PID_control import PID_W_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, AndTransition
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


class WingPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.BALL_Y_MIN = 0.05
        self.BALL_Y_MAX = 1.25

    def get_name(self):
        return f"<{self.robot.get_name()} Wing Attacker Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        self.robot.strategy.controller = controller(self.robot)

    def start(self):
        self.limit_cycle = LimitCycle(self.match)
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def update(self):
        target = [self.match.ball.x, self.match.ball.y]
        robot = (self.robot.x, self.robot.y)

        target[1] = min(self.BALL_Y_MAX, target[1])
        target[1] = max(self.BALL_Y_MIN, target[1])

        self.limit_cycle.set_target(target)
        self.limit_cycle.add_obstacle(self.get_virtual_obstacle(target))
        return self.limit_cycle.compute(self.robot, fitness=20)

    def get_virtual_obstacle(self, target):
        """
        - m:    angle of the line perpendicular to the line between the ball and
                the center of the goal
        - p:    distance of the virtual obstacles to the ball / radius of the virtual obstacle
        - vo:   virtual obstacle
        - j:    this is the angle between the ball and the center of the goal
        - m:    the normal angle perpendicular to j
        - r:    radius of the ball
        """
        aim_point = [self.field_w, self.BALL_Y_MAX] if target[1] > .65 else [self.field_w, self.BALL_Y_MIN]

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

        return virtual_obstacle


class CrossPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Angle Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        controller_kwargs = {'V_MAX': 0}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        if self.robot.y > .65:
            ang_diff = self.robot.theta - math.pi
        else:
            ang_diff = self.robot.theta + math.pi

        x = self.robot.x + 0.5*math.cos(ang_diff)
        y = self.robot.y + 0.5*math.sin(ang_diff)

        return x, y

    def start(self):
        pass

class MainStriker(Strategy):
    def __init__(self, match, name="Main_Attacker"):
        super().__init__(match, name, controller=PID_W_control)

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        field_dim = self.match.game.field.get_dimensions()

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        main = MainPlay(self.match, self.robot)
        main.start()
        wing = WingPlay(self.match, self.robot)
        wing.start()
        cross = CrossPlay(self.match, self.robot)
        cross.start()

        self.playerbook.add_play(main)
        self.playerbook.add_play(wing)
        self.playerbook.add_play(cross)

        on_wing = OnInsideBox(self.match, [0, 0.3, 1.5, 0.7], True)
        on_center = OnInsideBox(self.match, [0, 0.3, 1.5, 0.7])
        on_cross_region = OnInsideBox(self.match, [0, 0, 1.5, .15])
        on_near_ball = OnNextTo(self.match.ball, self.robot, 0.1)

        main.add_transition(on_wing, wing)
        wing.add_transition(on_center, main)
        wing.add_transition(AndTransition([on_near_ball, on_cross_region]), cross)
        cross.add_transition(on_center, main)

        # Estado inicial
        self.playerbook.set_play(main)

    def decide(self):
        res = self.playerbook.update()
        return res
