import math
import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector, distance

import json
import numpy as np

class Attacker(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match)

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

        """
        Essa é uma definição basica de campo, você sempre irá usar o objeto 
        PotentialField, sera passado para ele a referencia da partida (self.match) e um nome,
        nesse caso convencionamos que sera 'NOME_CLASSE|NOME_COMPORTAMENTO', o nome da classe
        já é dado pelo cósigo e o nome do comportamento você decide, nesse caso é FieldBehaviour
        """
        self.field = algorithims.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
        )
        """
        Crie quantos você quiser, cada um irá representar um comportamento que você definiu no passo (1)
        """



    def start(self, robot=None):
        super().start(robot=robot)

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

            def s(m):
                robot_pos = [m.robots[robot_id].x, m.robots[robot_id].y]
                ball_pos = [m.ball.x, m.ball.y]
                goal_pos = [0.75, 0.65]

                dist = distance(goal_pos, ball_pos, robot_pos)
                print(dist)

                weight = 1/2 + 1/2 * min((dist/0.2), 1)

                return weight * 0.75 if m.ball.y < 0.65 else 0
            
            return s
        
        def pr(self):
            robot_id = self.robot.robot_id

            def s(m):
                robot_pos = [m.robots[robot_id].x, m.robots[robot_id].y]
                ball_pos = [m.ball.x, m.ball.y]
                goal_pos = [0.75, 0.65]

                dist = distance(goal_pos, ball_pos, robot_pos)
                print(dist)

                weight = 1/2 + 1/2 * min((dist/0.2), 1)

                return weight * 0.75 if m.ball.y >= 0.65 else 0
            
            return s
            

        self.field.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 , 
                    m.ball.y + math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.10,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = pl(self)
            )
        )

        self.field.add_field(
            algorithims.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2 , 
                    m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))+ math.pi/2)*0.2
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.10,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = pr(self)
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
        behaviour = self.field

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)
            return (0, 0)

        return behaviour.compute([self.robot.x, self.robot.y])

