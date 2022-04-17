import algorithms
import math
import controller
from controller.PID import Robot_PID
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class Scratch(Strategy):
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
        self.field = algorithms.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
        )
        """
        No Start você ira adicionar campos potenciais aos comportamentos criados no metodo __init__
        de uma olhada na documentação de como se cria cada campo e como eles se comportam. Aqui, por exemplo
        tem um campo que leva a bola, note que elementos dinamicos podem ser passados como uma função lambda
        referencia util para funções lambdas: https://realpython.com/python-lambda/.
        """
        # self.field.add_field(
        #     algorithms.fields.PointField(
        #         self.match,
        #         target = lambda m: (0.75, 0.65),
        #         radius = 0.25, # 1cm
        #         decay = lambda x: x**2,
        #         multiplier = 0.5 # 50 cm/s
        #     )
        # )

        # self.field.add_field(
        #     algorithms.fields.LineField(
        #         self.match,
        #         target = (0.75, 0.65),
        #         theta = math.pi/2,
        #         line_size = 0.5,
        #         line_dist = 0.25,
        #         line_dist_max = 0.5,
        #         decay = lambda x: x,
        #         multiplier = 0.5
        #     )
        # )

        """
        Behaviour to make goalkeeper follow the ball vertically
        """

        def set_boundaries(m):
            x = 0.26
            y = m.ball.y
            if m.ball.y > 0.96:
                y = 0.96
            elif m.ball.y < 0.58:
                y = 0.58
            return x, y
            

        self.field.add_field(
            algorithms.fields.LineField(
                self.match,
                target = set_boundaries,
                theta = 0,
                line_size = 0.2,
                line_dist = 0.25,
                line_dist_max = 1.3,
                decay = lambda x: x**2,
                multiplier = 0.4,
            )
        )

        self.field.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.30, 0.75),
                theta = math.pi/2,
                line_size = 1.3,
                line_dist = 0.4,
                decay = lambda x: x,
                multiplier = 0.5,
            )
        )

        self.field.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 0.75),
                theta = math.pi/2,
                line_size = 1.3,
                line_dist = 0.4,
                line_dist_max = 0.1,
                decay = lambda x: 1,
                multiplier = -1,
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
        print(">>>>>>>>>>> ROBOT POS::", self.robot.x, self.robot.y, self.robot.robot_id)
        print(">>>>>>>>>>> ROBOT SPEED::", self.robot.vx, self.robot.vy, self.robot.vtheta)
        print(">>>>>>>>>>> BALL POS <<<<<<<<::", self.match.ball.x, self.match.ball.y)
        behaviour = self.field
        return behaviour.compute([self.robot.x, self.robot.y])

