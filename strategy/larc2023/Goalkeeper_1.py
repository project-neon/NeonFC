import math
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, AndTransition, OrTransition, \
    NotTransition, OnNextTo
from entities.plays.playbook import Trigger
from controller import PID_control, PID_W_control, UniController
from commons.math import point_in_rect
from algorithms import UnivectorField


class FollowBallPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15

        self.goal_left = self.field_h / 2 + .2
        self.goal_right = self.field_h / 2 - .2

    def get_name(self):
        return f"<{self.robot.get_name()} Follow Ball>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        pass

    def update(self):
        ball = self.match.ball

        projection_rate = 0.5 #(ball.x - .15) / (1 - .15)

        # projection_limit = 0.15*projection_rate

        projection_point = ball.y + projection_rate * ball.vy

        # bounded_projection = min( max( projection_point, ball.y - projection_limit), ball.y + projection_limit )

        y = min(max(projection_point, self.goal_right), self.goal_left)

        #Follow ball in a more agressive way
        #Considerations: X changes while ball more far away, not going from 0.04 to 0.4 suddenly; make changes more expressive in x as from each border of the goal goes to x_max-0.5
        if ball.x >= .75:
            x_max = 0.4
            x_min = 0.25

            x = math.sqrt((x_max**2)*(1- (((y-0.65)/0.65)**2)*(1-(x_min/x_max)**2)))
        else:
            x = .04

        return x, y


class InsideArea(PlayerPlay):
    def __init__(self, match, robot):
        # super().__init__(match, "Main_Attacker", controller=PID_W_control)
        super().__init__(match, robot)
        self.dl = 0.000001

    def get_name(self):
        return f"<{self.robot.get_name()} Inside Area Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        self.robot.strategy.controller = controller(self.robot, W_MAX=10)

    def start(self):
        self.univector = UnivectorField(n=6, rect_size=.1)
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def update(self):
        ball = self.match.ball
        theta_ball = math.atan2(0.3 - ball.y, 0.15 - ball.x) if ball.y < self.robot.y else math.atan2(1 - ball.y, 0.15 - ball.x)
        ball_rx, ball_ry = ball.x + .05 * math.cos(theta_ball), ball.y + .05 * math.sin(theta_ball)

        self.univector.set_target(g=(ball.x, ball.y), r=(ball_rx, ball_ry))

        x, y = self.robot.x, self.robot.y

        theta_d = self.univector.compute((x, y))
        theta_f = self.univector.compute(
            (x + self.dl * math.cos(self.robot.theta),
             y + self.dl * math.sin(self.robot.theta))
        )

        return ball.x, ball.y


class Spin(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Spin Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        controller_kwargs = {'V_MAX': 0, 'V_MIN': 0, 'W_MAX': 100000000000, 'KP':-10000000}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        if self.robot.y > .65:
            ang_diff = self.robot.theta - math.pi/2.1
        else:
            ang_diff = self.robot.theta + math.pi/2.1

        x = self.robot.x + 0.5*math.cos(ang_diff)
        y = self.robot.y + 0.5*math.sin(ang_diff)

        return x, y

    def start(self):
        pass


class Rest(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.target = (0.04, .65)
    def get_name(self):
        return f"<{self.robot.get_name()} Rest>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        pass

    def update(self):

        return self.target


class Goalkeeper_Prepare(Strategy):   #Goalkeeper that prepares ball for counter attack and stays as GK for the whole game
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_RSM2023", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        follow_ball = FollowBallPlay(self.match, self.robot)  # 3
        follow_ball.start()

        inside_area = InsideArea(self.match, self.robot)
        inside_area.start()

        spin = Spin(self.match, self.robot)
        spin.start()

        rest = Rest(self.match, self.robot)
        rest.start()

        self.playerbook.add_play(follow_ball)
        self.playerbook.add_play(inside_area)
        self.playerbook.add_play(spin)
        self.playerbook.add_play(rest)

        on_near_ball = OnNextTo(self.match.ball, self.robot, 0.09)   
        off_near_ball = OnNextTo(self.match.ball, self.robot, 0.12, True) #Fica checando se a bola ta perto ou não

        follow_ball.add_transition(OnInsideBox(self.match, [-.5, .3, .65, .7]), inside_area)  
        follow_ball.add_transition(on_near_ball, spin)
        follow_ball.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9]), rest)

        inside_area.add_transition(OnInsideBox(self.match, [-.5, .3, .75, .8], True), follow_ball)
        inside_area.add_transition(on_near_ball, spin)

        spin.add_transition(off_near_ball, follow_ball)
        rest.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9], True), follow_ball) #Primeiro é o trigger, segundo é a play que vai

        if self.playerbook.actual_play == None:
            self.playerbook.set_play(follow_ball)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        print(self.playerbook.actual_play)
        return res
