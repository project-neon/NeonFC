import math
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, AndTransition
from controller import PID_control, PID_W_control, NoController
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

        return 0.06, y


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
        pass

    def update(self):
        ball = self.match.ball

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
        if self.robot.y > self.match.ball.y:
            w = 1_000
        else:
            w = -1_000

        return 0, w

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


class Goalkeeper(Strategy):
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
        off_near_ball = OnNextTo(self.match.ball, self.robot, 0.12, True)

        follow_ball.add_transition(OnInsideBox(self.match, [-.5, .3, .65, .7]), inside_area)
        follow_ball.add_transition(on_near_ball, spin)
        follow_ball.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9]), rest)

        inside_area.add_transition(OnInsideBox(self.match, [-.5, .3, .75, .8], True), follow_ball)
        inside_area.add_transition(
            AndTransition([
                OnInsideBox(self.match, [-.5, .3, .65, .7], True),
                off_near_ball])
            , follow_ball)
        inside_area.add_transition(on_near_ball, spin)

        spin.add_transition(off_near_ball, follow_ball)
        rest.add_transition(OnInsideBox(self.match, [.75, -.3, 7, 1.9], True), follow_ball)

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
