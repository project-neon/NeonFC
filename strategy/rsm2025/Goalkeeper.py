import math

from entities.plays.playbook import Trigger
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, CheckAngle, AndTransition, RobotOnInsideBox, NotTransition, OnStuckTrigger, RobotLookBall
from controller import PID_control, PID_W_control, UniController, NoController
from NeonPathPlanning import UnivectorField, Point

import time

class IsInLine(Trigger):
    """
    Verifica se o robô está arbitrariamente perto de uma linha
    reta entre a bola e alguma determinada coordenada (que em tese é pra ser o centro do gol)
    A ideia é que caso o robô consiga se inserir entre o gol (nosso) e a bola, ele vai atacar
    pra cima da bola.
    """

    def calc_dist(self):
        # equação distância ponto-hiperplano
        # d=|w.p + b| / ||w||
        b = self.point
        #TODO
        pass

    def __init__(self, robot, ball, point, dist):
        super().__init__()
        self.robot = robot
        self.ball = ball
        self.point = point
        self.dist = dist

    def evaluate(self, *args, **kwargs):
        if (self.robot.theta > 0.9 and self.robot.theta < 2.3 ) and self.ball.y > .65:
            return True
        if (self.robot.theta > -2.3 and self.robot.theta < -0.9) and self.ball.y <= .65:
            return True
        return False

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

        cx = ball.x - dist; cy = ball.y - .75 # vetor que aponta do centro até a bola

        mag = (cx ** 2 + cy ** 2) ** .5

        cx /= mag; cy /= mag # normaliza o vetor c

        return cx * dist, cy * dist # TODO minimiza o raio caso a bola esteja dentro da área

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
            return .05, ball.y
        else:
            return .05, ball.y

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
        goal = [0,0.65]

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

        defend_play = DefendPlay(self.match, self.robot,.2,.1) #  TODO muda os parametros aqui
        follow_ball = FollowBallPlay(self.match, self.robot)
        
        self.playerbook.add_play(follow_ball)
        
        if self.playerbook.actual_play is None:
            self.playerbook.set_play(follow_ball)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def get_play(self):
        return self.playerbook.actual_play

    def decide(self):
        res = self.playerbook.update()
        # if self.old_play != self.playerbook.actual_play:
        # print(self.playerbook.actual_play)
        print('GK_id:', self.robot.robot_id)
            # self.old_play = self.playerbook.actual_play
        return res
