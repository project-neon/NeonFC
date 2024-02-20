import numpy as np
from NeonPathPlanning import UnivectorField, VSSS, Point
import math
from controller import PID_W_control, PID_control, UniController, NoController
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, AndTransition


class MainPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.dl = 0.000001

    def get_name(self):
        return f"<{self.robot.get_name()} Main Attacker Planning>"

    def start_up(self):
        super().start_up()
        controller = UniController
        self.robot.strategy.controller = controller(self.robot)

    def start(self):
        self.univector = UnivectorField(n=6, rect_size=.1, field=VSSS)

    def update(self):
        ball = self.match.ball
        guide = math.atan2(0.65 - ball.y, 1.6 - ball.x)

        self.univector.set_target(target=ball, guide=guide, guide_type='a')

        robot =  self.robot

        theta_d = self.univector.compute(robot)
        theta_f = self.univector.compute(Point(
            robot.x + self.dl * math.cos(robot.theta),
            robot.y + self.dl * math.sin(robot.theta)
        ))

        return theta_d, theta_f


class WingPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.dl = 0.000001

        self.BALL_Y_MIN = 0.05
        self.BALL_Y_MAX = 1.25

    def get_name(self):
        return f"<{self.robot.get_name()} Wing Attacker Planning>"

    def start_up(self):
        super().start_up()
        controller = UniController
        self.robot.strategy.controller = controller(self.robot)

    def start(self):
        self.univector = UnivectorField(n=6, rect_size=.1, field=VSSS)

    def update(self):
        ball = self.match.ball

        target = Point(ball.x, max(self.BALL_Y_MIN, min(self.BALL_Y_MAX, ball.y)))

        self.univector.set_target(target=target, guide=math.pi/2, guide_type='a')

        robot =  self.robot

        theta_d = self.univector.compute(robot)
        theta_f = self.univector.compute(Point(
            robot.x + self.dl * math.cos(robot.theta),
            robot.y + self.dl * math.sin(robot.theta)
        ))

        return theta_d, theta_f


class CrossPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Cross Planning>"

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

    def start(self):
        pass


class Wait(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Position Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs={'V_MIN': 0, 'K_RHO': 1.5}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        # print(self.position())
        return self.position()

    def start(self):
        pass

    def position(self):
        a = (.35, 1.1)
        b = (.35, 0.2)

        c = (self.robot.x, self.robot.y)

        d = [(r.x, r.y) for r in self.match.robots if r.strategy.name not in ["Main_Attacker", "Goalkeeper_LARC2023"]][0]

        # Calculate the distances between each robot and each fixed point
        distance_c_a = math.sqrt((c[0] - a[0]) ** 2 + (c[1] - a[1]) ** 2)
        distance_c_b = math.sqrt((c[0] - b[0]) ** 2 + (c[1] - b[1]) ** 2)
        distance_d_a = math.sqrt((d[0] - a[0]) ** 2 + (d[1] - a[1]) ** 2)
        distance_d_b = math.sqrt((d[0] - b[0]) ** 2 + (d[1] - b[1]) ** 2)

        # Assign the robots to the closer fixed point
        if distance_c_a + distance_d_b < distance_c_b + distance_d_a:
            return a
        else:
            return b


class LookAtBall(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Looking at the ball>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        controller_kwargs = {'V_MIN': 0, 'V_MAX': 0}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.match.ball.x, self.match.ball.y

    def start(self):
        pass


class MainStriker(Strategy):
    def __init__(self, match, name="Main_Attacker"):
        super().__init__(match, name, controller=PID_W_control)

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        main = MainPlay(self.match, self.robot)
        main.start()
        wing = WingPlay(self.match, self.robot)
        wing.start()
        cross = CrossPlay(self.match, self.robot)
        cross.start()
        defensive = Wait(self.match, self.robot)
        defensive.start()
        angle = LookAtBall(self.match, self.robot)

        self.playerbook.add_play(main)
        self.playerbook.add_play(wing)
        self.playerbook.add_play(cross)
        self.playerbook.add_play(defensive)
        self.playerbook.add_play(angle)

        on_wing = OnInsideBox(self.match, [0, 0.2, 1.5, 0.9], True)
        off_wing = OnInsideBox(self.match, [0, 0.25, 1.5, 0.8], False)
        on_cross_region = OnInsideBox(self.match, [1.35, -0.5, .2, 1.4], False)
        off_cross_region = OnInsideBox(self.match, [1.35, -0.5, .2, 1.4], True)
        on_near_ball = OnNextTo(self.match.ball, self.robot, 0.1, False)
        off_near_ball = OnNextTo(self.match.ball, self.robot, 0.1, True)
        on_defensive_box = OnInsideBox(self.match, [-.2, .3, .35, .7], False)
        off_defensive_box = OnInsideBox(self.match, [-.2, .3, .35, .7], True)
        on_position_1 = OnNextTo([.35, 1.1], self.robot, 0.1, False)
        off_position_1 = OnNextTo([.35, 1.1], self.robot, 0.1, True)
        on_position_2 = OnNextTo([.35, .2], self.robot, 0.1, False)
        off_position_2 = OnNextTo([.35, .2], self.robot, 0.1, True)

        main.add_transition(on_wing, wing)
        wing.add_transition(off_wing, main)

        wing.add_transition(AndTransition([on_near_ball, on_cross_region]), cross)
        cross.add_transition(off_cross_region, wing)
        cross.add_transition(off_near_ball, main)
        cross.add_transition(off_wing, main)

        main.add_transition(on_defensive_box, defensive)
        defensive.add_transition(off_defensive_box, main)

        defensive.add_transition(on_position_1, angle)
        defensive.add_transition(on_position_2, angle)
        angle.add_transition(AndTransition([off_position_1, off_position_2]), defensive)
        angle.add_transition(off_defensive_box, main)

        # Estado inicial
        self.playerbook.set_play(main)

    def decide(self):
        res = self.playerbook.update()
        return res
