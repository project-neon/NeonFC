from NeonPathPlanning import UnivectorField, Point
import math
from controller.uni_controller import UniController
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlaybook, PlayerPlay, OnInsideBox, OnNextTo, AndTransition
from commons.math import distance_between_points
from controller.PID_control import PID_control, PID_W_control


class MainPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.dl = 0.000001

    def get_name(self):
        return f"<{self.robot.get_name()} Shadow Position Planning>"

    def start_up(self):
        super().start_up()
        controller = UniController
        controller_kwargs = {'control_speed': True}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
        self.univector = UnivectorField(n=.3, rect_size=.1)

    def update(self):
        ball = self.match.ball
        robot =  self.robot
        main_st = next(filter(lambda r:r.strategy.name == "Main_Attacker", self.match.robots))
        obs_radius = distance_between_points(main_st, ball)
        gk = next(filter(lambda r:r.strategy.name == "Goalkeeper", self.match.robots))

        # for r in self.match.robots:
        #     if r != robot:
        #         self.univector.add_obstacle(r, obs_radius)


        # second attacker offset on x based on the distance of the main attacker to the ball
        # second attacker offset on y based on the distance of the ball to the center
        target = Point(max(main_st.x*.8, 0.2), main_st.y + .7*(.65-ball.y))

        self.univector.set_target(target, ball)
        self.univector.add_obstacle(main_st, obs_radius)
        self.univector.add_obstacle(gk, 0.075*1.4 + 0.1)


        theta_d = self.univector.compute(robot)
        theta_f = self.univector.compute(Point(
            robot.x + self.dl * math.cos(robot.theta),
            robot.y + self.dl * math.sin(robot.theta)
        ))

        self.robot.strategy.controller.target = target

        return theta_d, theta_f


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
        return self.position()

    def position(self):
        a = (.35, 1.1)
        b = (.35, 0.2)

        c = (self.robot.x, self.robot.y)
        d = [(r.x, r.y) for r in self.match.robots if r.strategy.name == "Main_Attacker"][0]

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

class ShadowAttacker(Strategy):
    def __init__(self, match, name="Shadow_Attacker"):
        super().__init__(match, name, controller=UniController, controller_kwargs={'control_speed': True})

    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        main = MainPlay(self.match, self.robot)
        defensive = Wait(self.match, self.robot)
        angle = LookAtBall(self.match, self.robot)

        self.playerbook.add_play(main)
        self.playerbook.add_play(defensive)
        self.playerbook.add_play(angle)

        on_defensive_box = OnInsideBox(self.match, [-.2, .3, .35, .7], False)
        off_defensive_box = OnInsideBox(self.match, [-.2, .3, .35, .7], True)
        on_position_1 = OnNextTo([.35, 1.1], self.robot, 0.1, False)
        off_position_1 = OnNextTo([.35, 1.1], self.robot, 0.1, True)
        on_position_2 = OnNextTo([.35, .2], self.robot, 0.1, False)
        off_position_2 = OnNextTo([.35, .2], self.robot, 0.1, True)

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
