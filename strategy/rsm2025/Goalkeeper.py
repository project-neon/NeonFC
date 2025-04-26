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

        # dist é a coordenada x do círculo de defesa
        # radius é o raio do círculo

        self.dist = dist
        self.radius = radius
        self.ball = self.match.ball
        self.goal_left = .3
        self.goal_right = 1.1

    def update(self):
        """Retorna o ponto mais próximo da bola
        no círculo com centro em (dist,0) de raio radius"""
        ball = self.ball
        dist = self.dist

        cx = ball.x - dist; cy = ball.y  # vetor que aponta do centro até a bola

        mag = (cx ** 2 + cy ** 2) ** .5

        cx /= mag; cy /= mag # normaliza o vetor c

        return cx * dist, cy * dist
        # eu quero testar se isso ^^^ dá certo, senão só descomenta tudo isso aqui embaixo

        # ball = self.ball
        #
        # if ball.y > 1.1:
        #     return .1, 1.1
        #
        # if ball.y < .3:
        #     return .1, .3
        #
        #
        # projection_rate = 0.3 #(ball.x - .15) / (1 - .15)
        # projection_point = ball.y + projection_rate * ball.vy
        #
        # y = min(max(projection_point, self.goal_right), self.goal_left)
        #
        # x = (0.0256 - y**2) **.2
        #
        # self.x = x, self.y = y
        #
        # return x, y
        
    # olha se isso é o controller certo     
    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

class FollowBallPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

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
            return ball.x, ball.y
        else:
            return ball.x, ball.y

class InterceptPlay(PlayerPlay):
    """Tenta colocar o robô entre a bola e o centro do gol"""
    def __init__(self, match, robot):
        super().__init__(match, robot)

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
        goal = [0,0] #presumo que isso seja a coordenada do gol,
        # não sou muito versado no sistema de coordenadas do NeonFC
        v = [goal[0] - ball.x, goal[1] - ball.y]
        # mag = (v[0]**2+v[1]**2)**.5
        # v[0]/= mag; v[1]/= mag
        v /= 2
        return v[0], v[1]


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
        print(self.playerbook.actual_play)
            # self.old_play = self.playerbook.actual_play
        return res
