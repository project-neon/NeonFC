import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np

class GoalKeeper(Strategy):
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
        
        self.normal_speed = 0.5
        self.push_speed = 0.8

        self.field_file = open('field.log', 'w')

        self.plot_field = plot_field

        self.base_rules = algorithims.fields.PotentialField(
            self.match,
            name="{}|BaseRules".format(self.__class__)
        )
        
        self.maintain = algorithims.fields.PotentialField(
            self.match, 
            name="{}|MaintainBehaviour".format(self.__class__)
        )

        self.alert = algorithims.fields.PotentialField(
            self.match, 
            name="{}|AlertBehaviour".format(self.__class__)
        )

        self.push = algorithims.fields.PotentialField(
            self.match, 
            name="{}|PushBehaviour".format(self.__class__)
        )


    def start(self, robot=None):
        super().start(robot=robot)

        def follow_ball(m):
            return (m.ball.x, m.ball.y)
        
        def quadratic(x):
            return x**2

        def inveterd_quadratic(x):
            return (-x**2) +1
        
        def constant(x):
            return 1

        def ball_speed(m):
            speed = min(max(
                m.ball.vy + 0.2,
                0.8
            ), 1)
            return speed

        self.base_rules.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0, 0.650),
                theta = math.pi/2,
                line_size = 0.2,
                line_size_max = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.2,
                line_dist_single_side = True,
                decay = inveterd_quadratic,
                multiplier = 1.0
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
                multiplier = 1.0
            )
        )

        self.maintain.add_field(self.base_rules)
        self.alert.add_field(self.base_rules)
        self.push.add_field(self.base_rules)
        
        self.maintain.add_field(
            algorithims.fields.PointField(
                self.match,
                target = (0 + 0.05, 0.650), # centro do campo
                radius = 0.05, # 30cm
                decay = quadratic,
                multiplier = 0.5 # 50 cm/s
            )
        )


        self.alert.add_field(
            algorithims.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = 1.8, # 30cm
                line_dist = 0.2,
                decay = inveterd_quadratic,
                multiplier = ball_speed # 50 cm/s
                )
        )

        self.alert.add_field(
            algorithims.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = 1.8, # 30cm
                line_dist = 0.05,
                decay = None,
                multiplier = 0.8 # 50 cm/s
            )
        )

        self.alert.add_field(
            algorithims.fields.LineField(
                self.match,
                target = (0 + 0.05, 0.650),
                theta = math.pi/2,
                line_size = 1.8, # 30cm
                line_dist = 0.05,
                decay = quadratic,
                multiplier = 0.5 # 50 cm/s
                )
        )

        self.push.add_field(
            algorithims.fields.PointField(
                self.match,
                target = follow_ball, # centro do campo
                radius = 0.05, # 30cm
                decay = constant,
                multiplier = 1 # 50 cm/s
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
        elif dist_to_ball <= 0.1: # 10cm
            behaviour = self.push
        else:
            behaviour = self.alert
        
        behaviour = self.alert
        print("BEHAVIOUR:", behaviour.name)
        X = []
        Y = []
        U = []
        V = []

        if self.plot_field:
            for x in range(-10, 150 + 10, 2):
                x = x/100.0
                for y in range(-10, 130 + 10, 2):
                    y = y/100.0
                    res = behaviour.compute([x, y])
                    X.append(x)
                    Y.append(y)
                    U.append(res[0])
                    V.append(res[1])

        plot_file = {
            "x": X,
            "y": Y,
            "u": U,
            "v": V,
            "robot_x": self.robot.x,
            "robot_y": self.robot.y,
            "ball_x": self.match.ball.x,
            "ball_y": self.match.ball.y,
            "behaviour": behaviour.name
        }

        self.field_file.write(json.dumps(plot_file) + "||")

        return behaviour.compute([self.robot.x, self.robot.y])
