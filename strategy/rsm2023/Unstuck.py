from strategy.BaseStrategy import Strategy
from controller import PID_W_control
from strategy.utils.player_playbook import PlayerPlaybook, PlayerPlay, Trigger, point_in_rect
import math


class Spin(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Spining>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        controller_kwargs = {'V_MAX': 0, 'V_MIN': 0, 'W_MAX': 100000000000, 'K_P': -100000000000}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        ang_diff = self.robot.theta + math.pi / 2.1

        x = self.robot.x + 0.5 * math.cos(ang_diff)
        y = self.robot.y + 0.5 * math.sin(ang_diff)

        return x, y

    def start(self):
        pass


class Reverse(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Reversing>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        self.robot.strategy.controller = controller(self.robot)

    def update(self):
        if self.robot.y > 1.25:
            return self.robot.x, 1.2
        if self.robot.y < .05:
            return self.robot.x, .1

        if self.robot.x > 1.45:
            return 1.4, self.robot.y
        if self.robot.x < .05:
            return .1, self.robot.y

    def start(self):
        pass


class OnCloseToWall(Trigger):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.box = [.1, .1, 1.3, 1.1]

    def evaluate(self, *args, **kwargs):
        return not point_in_rect([self.robot.x, self.robot.y], self.box)


class Unstuck(Strategy):
    def __init__(self, match, name="Unstuck"):
        super().__init__(match, name, controller=PID_W_control)

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        spin = Spin(self.match, self.robot)
        spin.start()
        reverse = Reverse(self.match, self.robot)
        reverse.start()

        self.playerbook.add_play(spin)
        self.playerbook.add_play(reverse)

        close_2_wall = OnCloseToWall(self.robot)

        spin.add_transition(close_2_wall, reverse)

        # Estado inicial
        self.playerbook.set_play(spin)

    def decide(self):
        res = self.playerbook.update()
        return res

