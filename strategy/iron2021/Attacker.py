import math
import algorithims
import controller
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector, distance

import json
import numpy as np

SPEED_FACTOR = 1.2

def point_in_rect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

class Attacker(Strategy):
    def __init__(self, match, plot_field=False, name="attacker"):
        super().__init__(match, name, controller=controller.TwoSidesLQR)

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
        
        self.plot_field = plot_field
        self.exporter = None

        self.obey_rules_speed = 0.5

        self.v_radius = 0.2
        self.v_radius_2 = 0.1

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

        self.seek = algorithims.fields.PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        self.defend = algorithims.fields.PotentialField(
            self.match,
            name="{}|DefendBehaviour".format(self.__class__)
        )

        self.carry = algorithims.fields.PotentialField(
            self.match, 
            name="{}|CarryBehaviour".format(self.__class__)
        )

        self.maintain = algorithims.fields.PotentialField(
            self.match, 
            name="{}|MaintainBehaviour".format(self.__class__)
        )

        self.base_rules = algorithims.fields.PotentialField(
            self.match,
            name="{}|BaseRulesBehaviour".format(self.__class__)
        )

        self.heading = algorithims.fields.PotentialField(
            self.match,
            name="{}|HeadingBehaviour".format(self.__class__)
        )

        if self.plot_field:
            self.exporter = algorithims.fields.PotentialDataExporter(self.robot.get_name())
        
        """
        No Start você ira adicionar campos potenciais aos comportamentos criados no metodo __init__
        de uma olhada na documentação de como se cria cada campo e como eles se comportam. Aqui, por exemplo
        tem um campo que leva a bola, note que elementos dinamicos podem ser passados como uma função lambda
        referencia util para funções lambdas: https://realpython.com/python-lambda/.
        """
        radius = 0.15 # 15cm

        def pl(self):
            robot_id = self.robot.robot_id
            radius = self.v_radius

            def s(m):
                robot_pos = [m.robots[robot_id].x, m.robots[robot_id].y]
                ball_pos = [m.ball.x, m.ball.y]
                goal_pos = [0.75, 0.65]

                dist = distance(goal_pos, ball_pos, robot_pos)

                weight = 1/2 + 1/2 * min((dist/radius), 1)

                return weight * 0.85 * SPEED_FACTOR if m.ball.y < 0.65 else 0
            
            return s
        
        def pr(self):
            robot_id = self.robot.robot_id
            radius = self.v_radius

            def s(m):
                robot_pos = [m.robots[robot_id].x, m.robots[robot_id].y]
                ball_pos = [m.ball.x, m.ball.y]
                goal_pos = [0.75, 0.65]

                dist = distance(goal_pos, ball_pos, robot_pos)

                weight = 1/2 + 1/2 * min((dist/radius), 1)

                return weight * 0.85 * SPEED_FACTOR if m.ball.y >= 0.65 else 0
            
            return s

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
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed
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
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed * 1.5
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
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed
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
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed * 1.5
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
                multiplier = self.obey_rules_speed * 1.75
            )
        )

        self.base_rules.add_field(
            algorithims.fields.LineField(
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
            algorithims.fields.LineField(
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
            algorithims.fields.LineField(
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
            algorithims.fields.LineField(
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
            algorithims.fields.LineField(
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
        
        self.seek.add_field(self.base_rules)

        def ttl(m):
            "target tangetial left"

            pos_x = (
                m.ball.x -
                math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025 -
                math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 + m.ball.vx/10
            )

            pos_y = (
                m.ball.y -
                math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025 -
                math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 + m.ball.vy/10
            )

            return (pos_x, pos_y)

        def ttr(m):
            "target tangetial left"

            pos_x = (
                m.ball.x -
                math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025 +
                math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.12 + m.ball.vx/10
            )

            pos_y = (
                m.ball.y -
                math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025 +
                math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.12 + m.ball.vy/10
            )

            return (pos_x, pos_y)

        self.seek.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=ttr,                                                                                                                                                                                                                                                                                                                                          
                radius = self.v_radius_2,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = pl(self)
            )
        )

        self.seek.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=ttl,                                                                                                                                                                                                                                                                                                                                          
                radius = self.v_radius_2,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = pr(self)
            )
        )

        # self.seek.add_field(
        #     algorithims.fields.PointField(
        #         self.match,
        #         target= lambda m: (m.ball.x, m.ball.y - 0.05),
        #         radius=0.15,
        #         radius_max=0.15,
        #         decay = lambda x: x,
        #         field_limits = [0.75* 2 , 0.65*2],
        #         multiplier = 1
        #     )
        # )

        self.seek.add_field(
            algorithims.fields.LineField(
                self.match,
                target= lambda m: (
                    m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025, 
                    m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025
                ),
                theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
                line_size = 1,
                line_size_single_side = True,
                line_dist = 0.15,
                line_dist_max = 0.15,
                decay = lambda x: x**2,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1 * SPEED_FACTOR # 75 cm/s
            )
        )

        self.seek.add_field(
            algorithims.fields.PointField(
                self.match,
                target= lambda m: (m.opposites[0].x, m.opposites[0].y),
                radius=0.25,
                radius_max=0.25,
                decay = lambda x: x-1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.2 * SPEED_FACTOR
            )
        )

        self.seek.add_field(
            algorithims.fields.PointField(
                self.match,
                target= lambda m: (m.opposites[1].x, m.opposites[1].y),
                radius=0.25,
                radius_max=0.25,
                decay = lambda x: x-1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.2 * SPEED_FACTOR
            )
        )

        self.seek.add_field(
            algorithims.fields.PointField(
                self.match,
                target= lambda m: (m.opposites[2].x, m.opposites[2].y),
                radius=0.25,
                radius_max=0.25,
                decay = lambda x: x-1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.2 * SPEED_FACTOR
            )
        )

        self.carry.add_field(self.base_rules)

        self.carry.add_field(
            algorithims.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.05, # 30cm
                decay = lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = lambda m: max(0.80, math.sqrt(m.ball.vx**2 + m.ball.vy**2) + 0.25) # 50 cm/s
            )
        )
        self.carry.add_field(
            algorithims.fields.LineField(
                self.match,
                target= lambda m: (
                    m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025, 
                    m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025
                ),
                theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
                line_size = 1,
                line_size_single_side = True,
                line_dist = 0.15,
                line_dist_max = 0.15,
                decay = lambda x: x**2,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1.3 # 75 cm/s
            )
        )

        self.maintain.add_field(self.base_rules)

        self.maintain.add_field(
            algorithims.fields.PointField(
                self.match,
                target = (0.40, 0.65),
                radius = 0.1,
                decay = lambda x: x,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.60
            )
        )

        self.defend.add_field(self.base_rules)

        self.defend.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0.05, 0.650),
                theta = math.pi/2,
                line_size = 0.25,
                line_size_max = 0.25,
                line_dist = 0.25,
                line_dist_max = 0.25,
                line_dist_single_side = True,
                decay = lambda x: (-x**2) + 1,
                multiplier = self.obey_rules_speed * 1.2
            )
        )

        self.defend.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + (math.cos(math.pi/3) if m.ball.y < 0.65 else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if m.ball.y < 0.65 else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.04,
                radius_max = 2,
                clockwise = lambda m: (m.ball.y < 0.65),
                decay=lambda x: 1 if x > 0.5 else 0.5,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1
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
        ball = [self.match.ball.x, self.match.ball.y]
        of_goal_area = [1.30, 0.30, 0.30, 0.70]
        goal_area = [-0.05, 0.30, 0.20, 0.70]

        min_angle_ball_to_goal = -math.atan2((self.match.ball.y - 0.55), (self.match.ball.x - 0.75*2))
        angle_robot_to_ball = -math.atan2((self.robot.y - self.match.ball.y), (self.robot.x - self.match.ball.x ))
        
        min_angle_to_goal = abs(min_angle_ball_to_goal - angle_robot_to_ball)

        dist_to_ball = math.sqrt(
            (self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2
        )

        dist_to_ball_goal = math.sqrt(
            (0 - self.match.ball.x)**2 + (0.65 - self.match.ball.y)**2
        )
        
        if point_in_rect(ball, of_goal_area):
            behaviour = self.carry
        elif point_in_rect(ball ,goal_area):
            behaviour = self.maintain
        elif dist_to_ball_goal <= 0.65 and self.match.ball.x <= 0.35:
            behaviour = self.defend
        elif (
            (ball[1] >= 1.1 and self.robot.y >= 1.05) 
            or 
            (ball[1] <= 0.2 and self.robot.y <= 0.25)) and self.robot.x < ball[0]:
            behaviour = self.carry
        elif (min_angle_to_goal <= 0.60) and (dist_to_ball <= 0.25):
            behaviour = self.carry
        else:
            behaviour = self.seek
        
        print(self.robot.get_name(), "::", behaviour.name)

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)
            return (0, 0)

        return behaviour.compute([self.robot.x, self.robot.y])

