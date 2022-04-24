from unicodedata import name

from cv2 import threshold
import algorithms
import math
import controller
from controller.PID import Robot_PID
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class Attacker(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, name="RSM-Attacker", controller=TwoSidesLQR, controller_kwargs={'l': 0.008})       
        """
        Ambiente para rascunhar novas estrategias com
        campos potencias:

        Para criar estrategias, você vai precisar:

        1) definir quantos comportamentos diferentes haverão na sua estrategia
        e quais serão as transições entre eles (usando posição da bola, angulo do robo, etc...)
        
        2) para cada comportamento, criar e adicionar os campos que você ira precisar para que
        ele ocorra como desejado, caso seja necessario visualizar, use o plot_field=True no começo
        dessa função para ter o arquivo de campo (deixe rodar alguns segundos apenas com a simulação ligada)

        3) Teste tanto os campos quanto as transições, analise se o campo não tem nenhum
        ponto morto (onde os campos se anulam).

        4) :opcional: rode com outros robos, para verificar se ele atrapalha outras estrategias, cogite adicionar
        comportamentos que evitem que ele cometa faltas ou fique travado na parede, atrapalhe o goleiro e principalmente
        que evite-o de fazer gol contra :P
        """

        """
        Essa é uma definição basica de campo, você sempre irá usar o objeto 
        PotentialField, sera passado para ele a referencia da partida (self.match) e um nome,
        nesse caso convencionamos que sera 'NOME_CLASSE|NOME_COMPORTAMENTO', o nome da classe
        já é dado pelo cósigo e o nome do comportamento você decide, nesse caso é FieldBehaviour
        """

        """
        Crie quantos você quiser, cada um irá representar um comportamento que você definiu no passo (1)
        """



    def start(self, robot=None):
        super().start(robot=robot)
        self.recover = algorithms.fields.PotentialField(
            self.match,
            name="{}|RecoverBehaviour".format(self.__class__)
        )
        self.push = algorithms.fields.PotentialField(
            self.match,
            name="{}|PushBehaviour".format(self.__class__)
        )


        def follow_ball(m):
            return (m.ball.x, m.ball.y)


        self.recover.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.15 , 
                    m.ball.y + math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.15
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.10,
                radius_max = 0.20,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = 0.3
            )
        )

        self.recover.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.15 , 
                    m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.15
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.10,
                radius_max = 0.20,
                clockwise = False,
                decay=lambda x: 1,
                multiplier = 0.3
            )
        )

        self.recover.add_field(
            algorithms.fields.PointField(
                self.match,
                target = follow_ball, 
                radius = 0.20,
                radius_max = 0.20,
                decay = lambda x: -1,
                multiplier = 0.3
            )
        )

        self.recover.add_field(
            algorithms.fields.PointField(
                self.match,
                target = follow_ball, 
                radius = 0.20,
                decay = lambda x: 1,
                multiplier = 0.3
            )
        )

        self.recover.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (-0.05, 0.65),
                theta = math.pi/2,
                line_size = 1.3,
                line_dist = 0.07,
                line_dist_max = 0.3,
                decay = lambda x: -1,
                multiplier = 0.6
            )
        )

        self.recover.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.15, 0.65),
                theta = 0,
                line_size = 0.15,
                line_dist = 0.07,
                decay = lambda x: -1,
                multiplier = 0.6
            )
        )

        self.push.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.20,
                decay = lambda x: x,
                multiplier = 0.4
            )
        )

        self.push.add_field(
            algorithms.fields.LineField(
                self.match,
                target=follow_ball,
                theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
                line_size = 1,
                line_size_single_side = True,
                line_dist = 0.15,
                line_dist_max = 0.15,
                decay = lambda x: x**2,
                multiplier = 0.4
            )
        )


    def swap_attacker(self):
        x = self.robot.x
        p = (0.65-self.match.ball.y)/(1.5-self.match.ball.x)
        r = math.pi/6
        sup = (p-r)*(x-self.match.ball.x)+self.match.ball.y
        inf = (p+r)*(x-self.match.ball.x)+self.match.ball.y
        if self.robot.y < sup and self.robot.y > inf:
            return True
        else:
            return False

    def on_ball(self):
        robot_x = self.robot.x
        robot_y = self.robot.y

        ball_x = self.match.ball.x
        ball_y = self.match.ball.y

        threshold = 0.05

        ball_in_front = ball_x > robot_x + 0.03
        ball_in_robot_range = abs(robot_y - ball_y) <= threshold

        return ball_in_front and ball_in_robot_range

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):
        """
        No decide iremos programar as regras que irão decidir qual 
        comportamento sera execuetado nesse momento. crie o conjunto de regras
        que preferir e no final atribua algum dos comportamentos a variavel behaviour
        """
        
        if self.swap_attacker() or self.on_ball():
            behaviour = self.push
        else:
            behaviour = self.recover
        print(behaviour.name)

        return behaviour.compute([self.robot.x, self.robot.y])

