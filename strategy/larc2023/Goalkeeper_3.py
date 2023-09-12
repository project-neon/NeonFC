import math
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, AndTransition, OrTransition, \
    NotTransition, OnNextTo
from entities.plays.playbook import Trigger
from controller import PID_control, PID_W_control, UniController
from commons.math import point_in_rect
from algorithms import UnivectorField

def genCircunferenceVector(vCoord,radius,middleDistance):
    field_w = 1 # <-- FIXME
    vCoord = max(-math.pi,min(vCoord/field_w*math.pi,math.pi))
    #vCoord /= radius
    cirCoords = {math.sin(vCoord),math.cos(vCoord)} * radius
    cirCoords[1] -= middleDistance
    return cirCoords

class StayInArea(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
    
    def start_up(self):
        super().start_up()
    
    def get_name(self):
        return f"<{self.robot.get_name()} Stay in area>"
    
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

    


class Goalkeeper (Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_RSM2023", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        print(self.playerbook.actual_play)
        return res
