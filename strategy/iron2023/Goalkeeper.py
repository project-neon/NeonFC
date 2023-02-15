import math
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import Trigger
from controller import SimpleLQR, TwoSidesLQR, UniController, PID_control
from algorithms import UnivectorField
from algorithms import LimitCycle
from algorithms.potential_fields.fields import PotentialField, PointField, LineField, TangentialField

class Returning_to_Goal(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Returning to Goal>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        self.robot.strategy.controller = controller(self.robot)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        return (.075, 0.65)

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_IRON2023", controller=TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        return
