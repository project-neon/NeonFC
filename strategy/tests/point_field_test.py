import algorithms
import math
import controller
from controller.PID import Robot_PID
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class PointTest(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "asdgasad", controller=TwoSidesLQR)


    def start(self, robot=None):
        super().start(robot=robot)
        self.point = algorithms.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
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
    
        self.point.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.75, 0.65),
                radius = 0.35,
                decay = lambda x: x**2,
                multiplier = 0.2
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
        
        behaviour = self.point
        
        return behaviour.compute([self.robot.x, self.robot.y])