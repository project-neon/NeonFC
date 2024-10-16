import math
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, CheckAngle, AndTransition, RobotOnInsideBox, NotTransition, OnStuckTrigger
from controller import PID_control, PID_W_control, UniController, NoController
from NeonPathPlanning import UnivectorField, Point

import time


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

    def update(self):
        ball = self.match.ball

        projection_rate = 0.5 #(ball.x - .15) / (1 - .15)

        # projection_limit = 0.15*projection_rate

        projection_point = ball.y + projection_rate * ball.vy

        # bounded_projection = min( max( projection_point, ball.y - projection_limit), ball.y + projection_limit )

        y = min(max(projection_point, self.goal_right), self.goal_left)

        if self.robot.y < .25: 
            return 0.04, .3
        if self.robot.y > 1.1:
            return 0.04, 1.0

        return 0.04, y


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
        self.univector = UnivectorField(n=6, rect_size=.1)

    def update(self):
        ball = self.match.ball
        theta_ball = math.atan2(0.3 - ball.y, 0.15 - ball.x) if ball.y < self.robot.y else math.atan2(1 - ball.y, 0.15 - ball.x)

        self.univector.set_target(target=ball, guide=theta_ball, guide_type='a')

        return ball.x, ball.y


class Spin(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Spin Planning>"

    def start_up(self):
        super().start_up()
        controller = NoController
        self.robot.strategy.controller = controller(self.robot)

    def update(self):
        if self.robot.y > .65:
            w = 1_000
        else:
            w = -1_000

        return 0, w


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

    def update(self):

        return self.target
    
class PushPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Push Play>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        ball = self.match.ball

        if ball.y > self.robot.y:
            return ball.x  , 1.3
        else:
            return ball.x , 0
        
class CorrectAngle(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.dl = 0.000001

    def get_name(self):
        return f"<{self.robot.get_name()} Correct Angle>"

    def start_up(self):
        super().start_up()
        controller = UniController
        self.robot.strategy.controller = controller(self.robot)

    def update(self):
        robot = self.robot

        theta_f = robot.theta

        if theta_f >= 3:
            theta_d = 4.8
        theta_d = 1.6

        return theta_d, theta_f
    
class Stuck(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Stuck Play>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

        self.t1 = time.time()

    def update(self):

        while self.t1 < 1:
            return self.robot.x + 2., self.robot.y
    

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        follow_ball = FollowBallPlay(self.match, self.robot)  # 3
        inside_area = InsideArea(self.match, self.robot)
        spin = Spin(self.match, self.robot)
        rest = Rest(self.match, self.robot)
        push = PushPlay(self.match, self.robot)
        correct_angle = CorrectAngle(self.match, self.robot)
        stuck = Stuck(self.match, self.robot)

        self.playerbook.add_play(follow_ball)
        self.playerbook.add_play(inside_area)
        self.playerbook.add_play(spin)
        self.playerbook.add_play(rest)
        self.playerbook.add_play(push)
        self.playerbook.add_play(correct_angle)
        self.playerbook.add_play(stuck)

        on_near_ball = OnNextTo(self.match.ball, self.robot, 0.09)
        off_near_ball = OnNextTo(self.match.ball, self.robot, 0.12, True)
        need_reposition = CheckAngle(self.robot, self.match.ball)
        not_need_reposition = NotTransition(need_reposition)
        on_stuck = OnStuckTrigger(self.robot)
        not_stuck = NotTransition(stuck)


        follow_ball.add_transition(OnInsideBox(self.match, [-.5, .3, .65, .7]), inside_area)
        follow_ball.add_transition(on_near_ball, spin)
        follow_ball.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9]), rest)
        follow_ball.add_transition(OnInsideBox(self.match, [-.5, -.1, .75, .3]), push)
        follow_ball.add_transition(OnInsideBox(self.match, [-.5, 1.05, .75, .4]), push)
        follow_ball.add_transition(AndTransition([RobotOnInsideBox(self.match, [-.5, .3, .65, .7], self.robot), need_reposition]), correct_angle)
        follow_ball.add_transition(on_stuck, stuck)

        push.add_transition(AndTransition([RobotOnInsideBox(self.match, [-.5, .3, .65, .7], self.robot), need_reposition]), correct_angle)
        spin.add_transition(AndTransition([RobotOnInsideBox(self.match, [-.5, .3, .65, .7], self.robot), need_reposition]), correct_angle)
        rest.add_transition(need_reposition, correct_angle)
        rest.add_transition(on_stuck, stuck)

        inside_area.add_transition(OnInsideBox(self.match, [-.5, -.1, .75, .3]), push)
        inside_area.add_transition(OnInsideBox(self.match, [-.5, 1.05, .75, .4]), push)
        inside_area.add_transition(on_near_ball, spin)
        inside_area.add_transition(need_reposition, correct_angle)
        inside_area.add_transition(on_stuck, stuck)


        push.add_transition(OnInsideBox(self.match, [-.5, -.1, .75, 1.7], True), follow_ball)
        push.add_transition(on_stuck, stuck)
        correct_angle.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9], True), follow_ball)
        correct_angle.add_transition(not_need_reposition, follow_ball)
        stuck.add_transition(not_stuck, follow_ball)

        spin.add_transition(off_near_ball, follow_ball)
        rest.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9], True), follow_ball)

        if self.playerbook.actual_play is None:
            self.playerbook.set_play(follow_ball)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def get_play(self):
        return self.playerbook.actual_play

    def decide(self):
        res = self.playerbook.update()
        # print(res)
        return res
