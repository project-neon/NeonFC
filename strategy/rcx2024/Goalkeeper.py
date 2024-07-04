import math
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo
from controller import PID_control, PID_W_control, UniController
from NeonPathPlanning import UnivectorField, Point


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
            return ball.x, 1.3
        else:
            return ball.x, 0


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

        self.playerbook.add_play(follow_ball)
        self.playerbook.add_play(inside_area)
        self.playerbook.add_play(spin)
        self.playerbook.add_play(rest)
        self.playerbook.add_play(push)

        on_near_ball = OnNextTo(self.match.ball, self.robot, 0.09)
        off_near_ball = OnNextTo(self.match.ball, self.robot, 0.12, True)

        follow_ball.add_transition(OnInsideBox(self.match, [-.5, .3, .65, .7]), inside_area)
        follow_ball.add_transition(on_near_ball, spin)
        follow_ball.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9]), rest)
        follow_ball.add_transition(OnInsideBox(self.match, [-.5, -.1, .75, .3]), push)
        follow_ball.add_transition(OnInsideBox(self.match, [-.5, 1.05, .75, .4]), push)

        inside_area.add_transition(OnInsideBox(self.match, [-.5, -.1, .75, .3]), push)
        inside_area.add_transition(OnInsideBox(self.match, [-.5, 1.05, .75, .4]), push)
        inside_area.add_transition(on_near_ball, spin)

        push.add_transition(OnInsideBox(self.match, [-.5, -.1, .75, 1.7], True), follow_ball)

        spin.add_transition(off_near_ball, follow_ball)
        rest.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9], True), follow_ball)

        if self.playerbook.actual_play is None:
            self.playerbook.set_play(follow_ball)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        # print(res)
        # print(OnInsideBox(self.match, [-.5, -.2, .75, 1.7], True).evaluate())
        # print(self.match.ball.x, self.match.ball.y)
        return res
