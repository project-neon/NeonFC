import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class Attacker(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match)

        """
        Essa estrategia descreve a um goleiro base, simples, que
        pode ser usado como baseline para o desenvolvimento de 
        outros goleiros.

        Sua estrategia é constituida por alguns contextos que são
        mudados baseados em regras simples dentro do jogo.
        Cada contexto é um campo potencial que rege que tipo de
        movimentação o goleiro precisa fazer. Sendo elas:

        1) maintain: O goleiro se mantem estatico no centro do gol

        1) alert: O goleiro se move dentro do gol, na pequena
        area, seguindo a bola caso ela esteja dentro do eixo X do gol
        e ficando nas "traves" caso a bola esteja nas laterais e dentro
        da nossa area

        2) push: O goleiro empurra a bola caso ela esteja vindo em sua
        direção, saindo um pouco da pequena area

        A troca entre os contextos reside no metodo decide()
        """
        
        self.normal_speed = 0.65
        self.push_speed = 0.8

        self.plot_field = plot_field
        self.exporter = None

        self.base_rules = algorithims.fields.PotentialField(
            self.match,
            name="{}|BaseRulesBehaviour".format(self.__class__)
        )
        
        self.seek = algorithims.fields.PotentialField(
            self.match, 
            name="{}|SeekBehaviour".format(self.__class__)
        )

        self.carry = algorithims.fields.PotentialField(
            self.match, 
            name="{}|CarryBehaviour".format(self.__class__)
        )


    def start(self, robot=None):
        super().start(robot=robot)

        if self.plot_field:
            self.exporter = algorithims.fields.PotentialDataExporter(self.robot.get_name())

        def follow_ball(m):
            return (m.ball.x, m.ball.y)
        
        def quadratic(x):
            return x**2

        def avoid_opposite(m):
            return (m.opposites[0].x, m.opposites[0].y)
        
        def inveterd_quadratic(x):
            return -((-x**2) +1)
        
        def inveterd_quadratic_s(x):
            return -((-x**6) +1)

        self.tangential = algorithims.fields.TangentialField(
                self.match,
                target=follow_ball,
                radius = 0.06,
                radius_max = 0.30,
                clockwise = 1,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.7
            )

        self.carry.add_field(self.tangential)
        
        # self.carry.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target = follow_ball, # centro do campo
        #         radius = 0.2, # 10cm
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.75 # 75 cm/s
        #     )
        # )

        # self.carry.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target = follow_ball, # centro do campo
        #         radius = 0.2, # 10cm
        #         radius_max = 0.2, # 10cm
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.75 # 75 cm/s
        #     )
        # )

        # self.carry.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target = avoid_opposite, # centro do campo
        #         radius = 0.2, # 10cm
        #         radius_max = 0.2, # 10cm
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 1.5 # 75 cm/s
        #     )
        # )

        # self.carry.add_field(
        #     algorithims.fields.LineField(
        #         self.match,
        #         target=follow_ball,
        #         theta=math.pi,
        #         line_size = 0.12,
        #         line_size_max = 0.12,
        #         line_size_single_side=True,
        #         line_dist = 0.12,
        #         line_dist_max = 0.12,
        #         decay = inveterd_quadratic_s,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 1.5 # 75 cm/s
        #     )
        # )


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        
        # self.tangential.clockwise = (self.match.ball.y - self.robot.y) > 0

        behaviour = self.carry

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)

        return behaviour.compute([self.robot.x, self.robot.y])

