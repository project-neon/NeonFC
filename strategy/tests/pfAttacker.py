import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

def point_in_rect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

class Attacker(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match)

        """
        Essa estrategia descreve a um atacante base, simples, que
        pode ser usado como baseline para o desenvolvimento de 
        outros atacantes.

        Sua estrategia é constituida por alguns contextos que são
        mudados baseados em regras simples dentro do jogo.
        Cada contexto é um campo potencial que rege que tipo de
        movimentação o atacante precisa fazer. Sendo elas:

        1) maintain: O atacante se mantem estatico proximo a pequena area
        caso a bola esteja na pequena area, para evitar dupla defesa (dois robos 
        nossos estarem na pequena area enquanto a bola esta lá)

        1) seek: A etapa mais complexa do atacante, onde ele precisa ir para
        parte de trás da bola (em relação ao proprio gol). Pega um angulo 
        adequado e um distancia adequada para ir apra o estado de carry.

        2) carry: Após um certo valor de angulo e distancia em relação a bola
        estarem satisfeitos, empurra a bola em direção ao gol em ritmo
        acelerado

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

        self.maintain = algorithims.fields.PotentialField(
            self.match,
            name="{}|MaintainRulesBehaviour".format(self.__class__)
        )

        self.avoiance = algorithims.fields.PotentialField(
            self.match,
            name="{}|AvoianceBehaviour".format(self.__class__)
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

        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0.750, 0.650*2 - 0.1),
                theta = -2*math.pi,
                line_size = 0.750,
                line_size_max = 0.750,
                line_dist = 0.1,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                inverse = True,
                decay = lambda x: x**(0.5),
                multiplier = 1.5
            )
        )

        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0.750, 0.1),
                theta = 2*math.pi,
                line_size = 0.750,
                line_size_max = 0.750,
                line_dist = 0.1,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = 1.5
            )
        )

        # self.maintain.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target = lambda m: (0.30, m.ball.y), # centro do campo
        #         radius = 0.2, # 10cm
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.75 # 75 cm/s
        #     )
        # )

        # self.seek.add_field(self.base_rules)
        # self.seek.add_field(self.avoiance)

        # self.carry.add_field(self.base_rules)
        # self.carry.add_field(self.avoiance)

        # self.seek.add_field(
        #     algorithims.fields.TangentialField(
        #         self.match,
        #         target=lambda m: (
        #             m.ball.x + math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 , 
        #             m.ball.y + math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2
        #         ),                                                                                                                                                                                                                                                                                                                                          
        #         radius = 0.10,
        #         radius_max = 0.20,
        #         clockwise = True,
        #         decay=lambda x: 1,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = lambda m: 0.9
        #     )
        # )

        # self.seek.add_field(
        #     algorithims.fields.TangentialField(
        #         self.match,
        #         target=lambda m: (
        #             m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 , 
        #             m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2
        #         ),                                                                                                                                                                                                                                                                                                                                          
        #         radius = 0.10,
        #         radius_max = 0.20,
        #         clockwise = False,
        #         decay=lambda x: 1,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.9
        #     )
        # )
        
        # self.seek.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target = follow_ball, # centro do campo
        #         radius = 0.2, # 10cm
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.75 # 75 cm/s
        #     )
        # )

        # self.seek.add_field(
        #     algorithims.fields.LineField(
        #         self.match,
        #         target=follow_ball,
        #         theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
        #         line_size = 1,
        #         line_size_single_side = True,
        #         line_dist = 0.15,
        #         line_dist_max = 0.15,
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.8 # 75 cm/s
        #     )
        # )

        # self.seek.add_field(
        #     algorithims.fields.LineField(
        #         self.match,
        #         target=follow_ball,
        #         theta=lambda m: ( -math.atan2((m.ball.y - 0.65), (m.ball.x - 0.75*2))),
        #         line_size = 0.18,
        #         line_size_max = 0.18,
        #         line_size_single_side = True,
        #         line_dist = 0.05,
        #         line_dist_max = 0.05,
        #         decay = inveterd_quadratic_s,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 1 # 75 cm/s
        #     )
        # )

        # self.carry.add_field(
        #     algorithims.fields.LineField(
        #         self.match,
        #         target=follow_ball,
        #         theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
        #         line_size = 1,
        #         line_size_single_side = True,
        #         line_dist = 0.15,
        #         line_dist_max = 0.15,
        #         decay = quadratic,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.5 # 75 cm/s
        #     )
        # )

        # self.carry.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target = follow_ball, # centro do campo
        #         radius = 0.05, # 10cm
        #         decay = None,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = ball_speed # 75 cm/s
        #     )
        # )


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        ball = [self.match.ball.x, self.match.ball.y]
        goal_area = [-0.05, 0.35, 0.20, 0.70]

        angle_ball_to_goal = -math.atan2((self.match.ball.y - 0.65), (self.match.ball.x - 0.75*2))
        angle_robot_to_ball = -math.atan2((self.robot.y - self.match.ball.y), (self.robot.x - self.match.ball.x ))
        
        angle_to_goal = abs(angle_ball_to_goal - angle_robot_to_ball)

        dist_to_ball = math.sqrt(
            (self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2
        )

        if point_in_rect(ball, goal_area):
            behaviour = self.maintain
        elif (angle_to_goal <= 0.5) and (dist_to_ball <= 0.10):
            behaviour = self.carry
        else:
            behaviour = self.seek

        print(self.robot.get_name(),"::",behaviour.name)

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)

        return behaviour.compute([self.robot.x, self.robot.y])

