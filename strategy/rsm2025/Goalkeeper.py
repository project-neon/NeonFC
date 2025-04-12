import math
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, CheckAngle, AndTransition, RobotOnInsideBox, NotTransition, OnStuckTrigger, RobotLookBall
from controller import PID_control, PID_W_control, UniController, NoController
from NeonPathPlanning import UnivectorField, Point

import time

class DefendPlay(PlayerPlay):
    radius = None
    dist = None
    def __init__(self,match,robot,dist,radius):
        super().__init__(match,robot)
        #dist é a distancia do ponto em relação a origem
        #radius é o raio de distancia ao ponto
        self.dist = dist
        self.radius = radius

    def update(self):
        x = self.robot.x; y = self.robot.y
        ball = self.match.ball;
        
        cx = ball.x - dist; cy = ball.y; #centro
        mag = (cx**2 + cy **2) ** .5
        cx /= mag; cy /= mag
        return cx * dist, cy * dist
        
    # olha se isso é o controller certo     
    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper", controller=PID_control)
        self.old_play = None

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        defend_play = DefendPlay(self.match, self.robot,6,4) #  TODO muda os parametros aqui
        
        self.playerbook.add_play(defend_play)
        
        if self.playerbook.actual_play is None:
            self.playerbook.set_play(defend_play)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def get_play(self):
        return self.playerbook.actual_play

    def decide(self):
        res = self.playerbook.update()
        # if self.old_play != self.playerbook.actual_play:
        print(self.playerbook.actual_play,self.old_play)
            # self.old_play = self.playerbook.actual_play
        return res
