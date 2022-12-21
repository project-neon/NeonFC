import algorithms
import math
import controller
from controller.PID import Robot_PID
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class Midfielder(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "asdgasad", controller=TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)
        self.field = algorithms.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
        )

        #campo potencial para seguir a posição/projeção da bola
        self.field.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (0.75, m.ball.y),
                radius = 0.25,
                decay = lambda x: x**2,
                multiplier = 0.4,
            )
        )

        # #campo potencial para repulsão de dentro do gol
        # self.field.add_field(
        #     algorithms.fields.LineField(
        #         self.match,
        #         target = (-0.05, 0.65),
        #         theta = math.pi/2,
        #         line_size = 1.3,
        #         line_dist = 0.07,
        #         line_dist_max = 0.45,
        #         decay = lambda x: - x**5,
        #         multiplier = 0.6
        #     )
        # )

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
        behaviour = self.field
        
        return behaviour.compute([self.robot.x, self.robot.y])

