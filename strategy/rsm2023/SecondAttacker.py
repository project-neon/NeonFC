from algorithms.univector_field import UnivectorField
import math
from controller.uni_controller import UniController
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlaybook, PlayerPlay, OnInsideBox, OnNextTo, AndTransition
from commons.math import distance_between_points
from controller.PID_control import PID_control, PID_W_control


class MainPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Shadow Position Planning>"

    def start_up(self):
        super().start_up()
        controller = UniController
        controller_kwargs = {'control_speed': True}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        ball = (self.match.ball.x, self.match.ball.y)
        main_st = [[i.x, i.y] for i in self.match.robots if i.strategy.name == "Main_Attacker"][0]
        obs_radius = distance_between_points(main_st, ball)
        target = main_st[:]
        gk = [[i.x, i.y] for i in self.match.robots if i.strategy.name == "Goalkeeper_IRON2023"][0]

        # second attacker offset on x based on the distance of the main attacker to the ball
        target[0] -= max(4*0.075, obs_radius)
        # second attacker offset on y based on the distance of the ball to the center
        target[1] += .5*(.65-ball[1])

        self.univector_field.set_target(target, ball)
        self.univector_field.del_obstacle(all=True)
        self.univector_field.add_obstacle(main_st, 0.075*1.4, obs_radius-0.075*1.4)
        self.univector_field.add_obstacle(gk, 0.075*1.4, 0.1)

        x, y = self.robot.x, self.robot.y
        theta_d = self.univector_field.compute((x, y))
        theta_f = self.univector_field.compute((x + self.dl * math.cos(self.robot.theta), y + self.dl * math.sin(self.robot.theta)))

        self.robot.strategy.controller.target = target

        return theta_d, theta_f

    def start(self):
        self.univector_field = UnivectorField(n=2, rect_size=0)
        self.dl = 0.000001


class Wait(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Position Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs={'V_MIN': 0, 'K_RHO': 75}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        print(self.position())
        return self.position()

    def start(self):
        pass

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

    def start(self):
        pass


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

class ShadowAttacker(Strategy):
    def __init__(self, match, name="Shadow_Attacker"):
        super().__init__(match, name, controller=UniController, controller_kwargs={'control_speed': True})

    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        main = MainPlay(self.match, self.robot)
        main.start()
        defensive = Wait(self.match, self.robot)
        defensive.start()
        angle = LookAtBall(self.match, self.robot)
        angle.start()

        self.playerbook.add_play(main)
        self.playerbook.add_play(defensive)
        self.playerbook.add_play(angle)

        on_defensive_box = OnInsideBox(self.match, [-.2, .3, .35, .7])
        on_positon_1 = OnNextTo([.35, 1.1], self.robot, 0.1)
        on_positon_2 = OnNextTo([.35, .2], self.robot, 0.1)

        main.add_transition(on_defensive_box, defensive)

        main.add_transition(AndTransition([on_positon_1, on_defensive_box]), angle)
        main.add_transition(AndTransition([on_positon_2, on_defensive_box]), angle)

        # Estado inicial
        self.playerbook.set_play(main)

    def decide(self):
        res = self.playerbook.update()
        return res
