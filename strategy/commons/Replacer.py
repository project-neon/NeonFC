from algorithms.univector_field import UnivectorField
import math
from controller.PID_control import PID_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlaybook, PlayerPlay, OnNextTo, OnFoul
from commons.math import distance_between_points

# class Replacer(Strategy):
#     def __init__(self, match, plot_field=False):
#         super().__init__(match, "Replacer", controller=PID_control, controller_kwargs={'V_MIN': 0, 'K_RHO': 50})
#
#     def start(self, robot=None):
#         super().start(robot=robot)
#
#     def reset(self, robot=None):
#         super().reset()
#         if robot:
#             self.robot = robot
#
#     def decide(self):
#         target = (.75, .65)
#
#         return target

class Position(PlayerPlay):
    def __init__(self, match, robot, target_dict):
        super().__init__(match, robot)
        self.target_dict = target_dict

    def get_name(self):
        return f"<{self.robot.get_name()} Position Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs={'V_MIN': 0, 'V_MAX':0.1 ,  'K_RHO': 25}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):

        return self.target_dict

    def start(self):
        pass

class Angle(PlayerPlay):
    def __init__(self, match, robot, angle):
        super().__init__(match, robot)
        self.target = angle

    def get_name(self):
        return f"<{self.robot.get_name()} Angle Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {'V_MIN': 0, 'V_MAX': 0, 'K_P': 100000}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.target

    def start(self):
        pass


class Replacer(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control)

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        field_dim = self.match.game.field.get_dimensions()

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        # Criando Path Planning baseado em Astar
        target = [0.75, 0.65]
        pos = Position(self.match, self.robot, target)
        pos.start()
        angle = Angle(self.match, self.robot, [1.5, 0.65])
        angle.start()

        self.playerbook.add_play(pos)
        self.playerbook.add_play(angle)

        on_point = OnNextTo(target, self.robot, 0.02)
        off_point = OnNextTo(target, self.robot, 0.02, True)

        pos.add_transition(on_point, angle)
        angle.add_transition(off_point, pos)

        # Estado inicial é o astar
        self.playerbook.set_play(pos)

    def decide(self):
        res = self.playerbook.update()
        return res
