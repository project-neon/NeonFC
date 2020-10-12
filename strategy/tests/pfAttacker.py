import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class Attacker(Strategy):
    def __init__(self, match, plot_field=True):
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
        
        def repulse(x):
            return (-x**2) +1
        
        def inveterd_quadratic_s(x):
            return -((-x**6) +1)
        
        def ball_speed(m):
            speed = min(max(
                m.ball.vy + 0.2,
                0.7
            ), 1)
            return speed

        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0, 0.650),
                theta = math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = repulse,
                multiplier = 0.8
            )
        )
        
        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (-0.2, 0.650),
                theta = 0,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = quadratic,
                multiplier = 1.2
            )
        )

        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (2*0.750, 0.650),
                theta = 3*math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = repulse,
                multiplier = 0.8
            )
        )
        
        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (2*0.750+0.2, 0.650),
                theta = 2*math.pi,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = quadratic,
                multiplier = 1.2
            )
        )

        self.seek.add_field(self.base_rules)
        self.carry.add_field(self.base_rules)

        self.seek.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 , 
                    m.ball.y + math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.10,
                radius_max = 0.20,
                clockwise = True,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.9
            )
        )

        self.seek.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 , 
                    m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.10,
                radius_max = 0.20,
                clockwise = False,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.9
            )
        )
        
        self.seek.add_field(
            algorithims.fields.PointField(
                self.match,
                target = follow_ball, # centro do campo
                radius = 0.2, # 10cm
                decay = quadratic,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.6 # 75 cm/s
            )
        )

        self.seek.add_field(
            algorithims.fields.LineField(
                self.match,
                target=follow_ball,
                theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
                line_size = 1,
                line_size_single_side = True,
                line_dist = 0.10,
                line_dist_max = 0.10,
                decay = quadratic,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.9 # 75 cm/s
            )
        )

        self.seek.add_field(
            algorithims.fields.LineField(
                self.match,
                target=follow_ball,
                theta=lambda m: ( -math.atan2((m.ball.y - 0.65), (m.ball.x - 0.75*2))),
                line_size = 0.18,
                line_size_max = 0.18,
                line_size_single_side = True,
                line_dist = 0.05,
                line_dist_max = 0.05,
                decay = inveterd_quadratic_s,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1 # 75 cm/s
            )
        )

        self.carry.add_field(
            algorithims.fields.PointField(
                self.match,
                target = follow_ball, # centro do campo
                radius = 0.05, # 10cm
                decay = None,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = ball_speed # 75 cm/s
            )
        )


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        angle_ball_to_goal = -math.atan2((self.match.ball.y - 0.65), (self.match.ball.x - 0.75*2))
        angle_robot_to_ball = -math.atan2((self.robot.y - self.match.ball.y), (self.robot.x - self.match.ball.x ))
        
        angle_to_goal = abs(angle_ball_to_goal - angle_robot_to_ball)

        dist_to_ball = math.sqrt(
            (self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2
        )

        if (angle_to_goal <= 0.5) and (dist_to_ball <= 0.10):
            behaviour = self.carry
        else:
            behaviour = self.seek
        
        print(behaviour.name)

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)

        return behaviour.compute([self.robot.x, self.robot.y])

