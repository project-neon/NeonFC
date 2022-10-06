import algorithms
import math
import controller
from controller.PID import Robot_PID
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np


class Goalkeeper(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "asdgasad", controller=TwoSidesLQR)

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
        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
        )

        self.calm = algorithms.fields.PotentialField(
            self.match,
            name="{}|CalmBehaviour".format(self.__class__)
        )

        self.edge = algorithms.fields.PotentialField(
            self.match,
            name="{}|EdgeBehaviour".format(self.__class__)
        )

        """
        No Start você ira adicionar campos potenciais aos comportamentos criados no metodo __init__
        de uma olhada na documentação de como se cria cada campo e como eles se comportam. Aqui, por exemplo
        tem um campo que leva a bola, note que elementos dinamicos podem ser passados como uma função lambda
        referencia util para funções lambdas: https://realpython.com/python-lambda/.
        """

        """
        Behaviour to make goalkeeper follow the ball vertically
        """

        def set_boundaries(m):
            x = 0.04
            g_hgr = 0.83
            g_lwr = 0.45

            x_rob = x + 0.075 / 2
            if m.ball.vx == 0:
                if m.ball.y > g_hgr:
                    y = g_hgr
                elif m.ball.y < g_lwr:
                    y = g_lwr
                else:
                    y = m.ball.y
                return (x, y)

            y = (m.ball.vy / m.ball.vx) * (x_rob - m.ball.x) + m.ball.y
            # y = m.ball.y + m.ball.vy*(4/30)

            mp = (0.65 + m.ball.y) / 2

            if m.ball.y > g_hgr:
                y = g_hgr
            elif m.ball.y < g_lwr:
                y = g_lwr
            else:
                if y > g_hgr or y < g_lwr:
                    if m.ball.y < 0.65:
                        y = ((mp - m.ball.y) / m.ball.x) * (x_rob - m.ball.x) + m.ball.y
                    elif m.ball.y > 0.65:
                        y = ((m.ball.y - mp) / m.ball.x) * (x_rob - m.ball.x) + m.ball.y
            return x, y

        def edging_point(m):
            x = 0.04
            g_hgr = 0.83
            g_lwr = 0.45

            if m.ball.y < 0.65:
                y = g_lwr - (0.075 / 2)
            elif m.ball.y > 0.65:
                y = g_hgr + (0.075 / 2)

            return x, y


        self.calm.add_field(
            algorithms.fields.PointField(
                self.match,
                target=(0.04, 0.68),
                radius=0.2,
                decay=lambda x: x ** 10,
                multiplier=0.4
            )
        )

        self.defend.add_field(
            algorithms.fields.PointField(
                self.match,
                target=lambda m: (m.ball.x, m.ball.y),
                radius=0.2,
                decay=lambda x: 1,
                multiplier=0.4
            )
        )

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

        behaviour = None

        if self.match.ball.x > 0.75:
            behaviour = self.calm
        else:
            behaviour = self.defend

        behaviour = self.defend

        return behaviour.compute([self.robot.x, self.robot.y])
        
