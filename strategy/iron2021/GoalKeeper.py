import math
import algorithms
import controller
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector, distance

import json
import numpy as np

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def proj_goaline(pos_ball, speed_ball):
    """
    pos_ball: posição da bola
    speed_ball: vetor velocidade da bola
    """
    if math.sqrt((speed_ball[0]**2 + speed_ball[1]**2)) <= 0.05:
        line1 = [pos_ball, [0, 0.65]]
    else:
        line1 = [pos_ball, [pos_ball[0] + speed_ball[0], pos_ball[1] + speed_ball[1]]]

    line2 = [[0.125, 0.65], [0.125, 0]]

    res = None
    try:
        res = line_intersection(line1, line2)[1]
    except:
        res = pos_ball[1]

    return res

class GoalKeeper(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(
            match, "goalkeeper", controller=controller.TwoSidesLQR)

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
        
        self.normal_speed = 0.95
        self.obey_rules_speed = 0.5
        self.push_speed = 0.8

        self.plot_field = plot_field
        self.exporter = None

        self.v_radius = 0.1
        self.v_radius_2 = 0.05

    def start(self, robot=None):
        super().start(robot=robot)

        self.base_rules = algorithms.fields.PotentialField(
            self.match,
            name="{}|BaseRulesBehaviour".format(self.__class__)
        )
        
        self.maintain = algorithms.fields.PotentialField(
            self.match, 
            name="{}|MaintainBehaviour".format(self.__class__)
        )

        self.alert = algorithms.fields.PotentialField(
            self.match, 
            name="{}|AlertBehaviour".format(self.__class__)
        )

        self.push = algorithms.fields.PotentialField(
            self.match, 
            name="{}|PushBehaviour".format(self.__class__)
        )

        if self.plot_field:
            self.exporter = algorithms.fields.PotentialDataExporter(self.robot.get_name())

        def follow_ball(m):
            return (m.ball.x, m.ball.y)

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 0.650),
                theta = math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed
            )
        )
        
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (-0.2, 0.650),
                theta = 0,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed * 1.5
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*0.750, 0.650),
                theta = 3*math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed
            )
        )
        
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*0.750+0.2, 0.650),
                theta = 2*math.pi,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed * 1.5
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
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
                multiplier = self.obey_rules_speed * 1.75
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.750, 0.1),
                theta = 2*math.pi,
                line_size = 0.750,
                line_dist = 0.1,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.75
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.075, 0.85),
                theta = math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.075, 0.0),
                theta = math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*(0.75)-0.075, 0.85 + 0.45),
                theta = 3*math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (2*(0.75)-0.075, 0.0 + 0.45),
                theta = 3*math.pi/2,
                line_size = 0.45,
                line_dist = 0.075,
                line_dist_max = 0.075,
                line_dist_single_side = True,
                line_size_single_side = True,
                decay = lambda x: x**(0.5),
                multiplier = self.obey_rules_speed * 1.8
            )
        )

        self.maintain.add_field(self.base_rules)
        self.alert.add_field(self.base_rules)
        self.push.add_field(self.base_rules)
        
        self.maintain.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0 + 0.075, 0.650), # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: x**2,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = self.normal_speed # 50 cm/s
            )
        )

        keep_behind_ball = algorithms.fields.LineField(
            self.match,
            target = follow_ball,
            theta = lambda m: ( -math.atan2((m.ball.y - 0.65), (m.ball.x - 0.75*2))),
            line_size = 0.18,
            line_size_max = 0.18,
            line_size_single_side = True,
            line_dist = 0.05,
            line_dist_max = 0.05,
            decay = lambda x: -((-x**6) +1),
            field_limits = [0.75* 2 , 0.65*2],
            multiplier = self.normal_speed # 75 cm/s
        )

        self.alert.add_field(keep_behind_ball)
        self.push.add_field(keep_behind_ball)

        def projection_ball(m):
            if m.ball.vy > 0:
                return (0.075, max(0.30, min(m.ball.y, 0.70 + 0.30)) )
            proj = proj_goaline(
                [m.ball.x, m.ball.y],
                [m.ball.vx, m.ball.vy]
            )
            proj_with_bars = (0.075, max(0.30, min(proj, 0.70 + 0.30)) )
            return proj_with_bars

        self.alert.add_field(
            algorithms.fields.PointField(
                self.match,
                target = projection_ball, # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.2 # 50 cm/s
            )
        )

        # self.push.add_field(
        #     algorithms.fields.PointField(
        #         self.match,
        #         target = lambda m : (m.ball.x - 0.10, m.ball.y),
        #         radius = 0.05, # 30cm
        #         decay = lambda x: 1,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = lambda m: 0.75 #max(0.80, math.sqrt(m.ball.vx**2 + m.ball.vy**2) + 0.5) # 50 cm/s
        #     )
        # )

        # self.push.add_field(
        #     algorithms.fields.PointField(
        #         self.match,
        #         target = lambda m : (m.ball.x, m.ball.y),
        #         radius = 0.05, # 5cm
        #         radius_max = 0.05, # 5cm
        #         decay = lambda x: -1,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 0.8 # 50 cm/s
        #     )
        # )

        self.push.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + (math.cos(math.pi/3) if m.ball.y < 0.65 else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if m.ball.y < 0.65 else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                             
                radius = self.v_radius_2,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.2
            )
        )

        self.push.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + (math.cos(math.pi/3) if m.ball.y < 0.65 else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if m.ball.y < 0.65 else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                              
                radius = self.v_radius_2,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.2
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        dist_to_ball = np.linalg.norm(
            np.array([self.robot.x, self.robot.y]) - 
            np.array([self.match.ball.x, self.match.ball.y])
        )
        
        behaviour = None

        if self.match.ball.x > 0.650:
            behaviour = self.maintain
        elif dist_to_ball <= 0.15 and self.robot.x < self.match.ball.x + self.robot.dimensions['L']: # 10cm
            behaviour = self.push
        else:
            behaviour = self.alert

        print('{}::{}'.format(self.robot.get_name(), behaviour.name))
        # print(self.match.ball.x)

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)

        return behaviour.compute([self.robot.x, self.robot.y])

