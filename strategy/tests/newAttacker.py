import math
#from random import betavariate
#from scipy.spatial.qhull import Voronoi
import algorithms
import numpy as np
import controller
from fields.field import Field as fd
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector, distance
from strategy import DebugTools
from algorithms.astar import AStar, Node
from algorithms.astar.fieldGraph import FieldGraph
from scipy.spatial import Voronoi
from controller import TwoSidesLQR

SPEED_FACTOR = 1.3
POSSESSION_DIST = 12 * 10**-2 # Distancia minima considerada pra posse de bola em cm
BALL_RADIUS = 2.135 * 10**-2 # Raio da bola em cm
OPPOSITE_GOAL_X = 1.5 # Coordenada x do goal adversário

# Orientantion lane of the robot to support ball possession verification
def robot_orientation_line(self, robots):
    for r in robots:
        m = math.tan(r.theta) # slope of the line
        xi, yi = r.x, r.y
        

        if r.get_name() == self.robot.get_name():
            robotPointAhead = [r.x]



#class newAttacker(DebugTools.DebugPotentialFieldStrategy):
class newAttacker(Strategy):
    def __init__(self, match):
        super().__init__(
            match, 'newAttacker1',
            controller=TwoSidesLQR) #revisar nome no futuro
        
        self.match = match

        #self.astar = algorithms.astar.AStar()

        self.obey_rules_speed = 0.5

        self.path = None

        self.goal_aim_y = 0.65 # point of the line of the goal that the robot is aiming at in kick behaviour

    # Method to verify if robot has the control of the ball
    def ball_control(self, robot):
        if (robot.y < self.match.ball.y-0.015) or (robot.y > self.match.ball.y+0.015):
            return True
        else:
            return False

    # Method for goal alignment verification
    def goal_aim(self, robot):
        if robot.theta >= math.pi/2 and robot.theta <= math.pi*(3/2):
            return False
        else:
            robot_line_m = math.tan(robot.theta)
            robot_line_n = robot.y - robot.x * robot_line_m # n=y-xm
            aimed_at_y = OPPOSITE_GOAL_X * robot_line_m + robot_line_n
            if aimed_at_y >= 0.49 and aimed_at_y <= 0.81:
                self.goal_aim_y = aimed_at_y
                return True
            return False
    
    # Método para calcular distância euclidiana até a bola
    def dist_to_ball(self, r, m):
        return math.dist((r.x, r.y), (m.ball.x, m.ball.y))

    #ATTENTION!
    # Quando mais de um robo estiver em posse qual o criterio de desempate
    # Se posse esta com um inimigo nao precisa iterar mais
    # Se robo estiver com um robo nosso iterar pra ver se tem tbm um oponente com a bola

    # Método para definir a posse da bola => 
    # 0 - posse do atacante, 1 - nosso time, 2 - outro time, 3 - posse de ninguem
    def get_ball_possession(self, robots, m):
        for r in robots:
            if r.get_name() == self.robot.get_name():
                if r.x < m.ball.x: # Ball ahead of robot
                    if(self.dist_to_ball(r, m) <= POSSESSION_DIST-0.05):
                        return 0
            elif r.team_color == self.robot.team_color:
                if(self.dist_to_ball(r, m) <= POSSESSION_DIST):
                    return 1
            else:
                if(self.dist_to_ball(r, m) <= POSSESSION_DIST):
                    return 2
        return 3

    def generate_graph(self, objective):
        self.graph = FieldGraph()

        self.robot_node = Node([self.robot.x, self.robot.y])
        self.graph.set_start(self.robot_node)

        self.robot_node.position = [self.robot.x, self.robot.y]

        corners = [
            [self.match.game.field.get_dimensions()[0], 0],
            [0, 0],
            [0, self.match.game.field.get_dimensions()[1]],
            [self.match.game.field.get_dimensions()[0], self.match.game.field.get_dimensions()[1]]
        ]
        obstacles = corners + [ [r.x, r.y] for r in self.match.opposites] + [
            [r.x, r.y] for r in self.match.robots 
            if r.robot_id != self.robot.robot_id
        ] + [ [objective[0], objective[1]], self.robot_node.position]

        vor = Voronoi(obstacles)

        target_node = Node([objective[0], objective[1]])

        nodes = [
            Node([a[0], a[1]]) for a in vor.vertices
        ] + [
            target_node, self.robot_node
        ]

        objective_index = len(obstacles) - 2
        robot_index = len(obstacles) - 1
        
        self.graph.set_nodes(nodes)

        polygon_objective_edges = []
        polygon_robot_edges = []

        for edge, ridge_vertice in zip(vor.ridge_vertices, vor.ridge_points):
            if edge[0] == -1: continue
            self.graph.add_edge([nodes[edge[0]], nodes[edge[1]]])

            if objective_index in ridge_vertice:
                polygon_objective_edges.append(nodes[edge[0]])
                polygon_objective_edges.append(nodes[edge[1]])

            if robot_index in ridge_vertice:
                polygon_robot_edges.append(nodes[edge[0]])
                polygon_robot_edges.append(nodes[edge[1]])
            
            if objective_index in ridge_vertice and robot_index in ridge_vertice:
                self.graph.add_edge([self.robot_node, target_node])

        for edge_to_objective in set(polygon_objective_edges):
            self.graph.add_edge([edge_to_objective, target_node])

        for edge_to_objective in set(polygon_robot_edges):
            self.graph.add_edge([edge_to_objective, self.robot_node])


        path = AStar(self.robot_node, target_node).calculate()

        return path[1]

    def reset(self, robot=None):
        super().reset()

        self.astar.reset()

        if robot:
            self.robot = robot

    def start(self, robot=None):
        super().start(robot=robot)
        self.base_rules = algorithms.fields.PotentialField(
            self.match,
            name="{}|BaseRules".format(self.__class__)
        )

        self.avoid_obstacles = algorithms.fields.PotentialField(
            self.match,
            name="{}|AvoidObstacles".format(self.__class__)
        )

        self.seek = algorithms.fields.PotentialField(
            self.match,
            name="{}|SeekBehavior".format(self.__class__)
        )

        self.seek_anti_clockwise = algorithms.fields.PotentialField(
            self.match,
            name="{}|AntiClockwiseSeekBehavior".format(self.__class__)
        )

        self.seek_clockwise = algorithms.fields.PotentialField(
            self.match,
            name="{}|ClockwiseSeekBehavior".format(self.__class__)
        )

        self.tackle = algorithms.fields.PotentialField(
            self.match,
            name="{}|TackleBehavior".format(self.__class__)
        )

        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|DefendBehavior".format(self.__class__)
        )

        self.kick = algorithms.fields.PotentialField(
            self.match,
            name="{}|KickBehavior".format(self.__class__)
        )

        self.carry = algorithms.fields.PotentialField(
            self.match,
            name="{}|CarryBehavior".format(self.__class__)
        )

        self.carry2 = algorithms.fields.PotentialField(
            self.match,
            name="{}|Carry2Behavior".format(self.__class__)
        )
        

        # Methods needed for the base rules 
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

        """
        Potential Fields
        """

	#ATTENTION parametrizar tudo do field

        # Line size -> em relação ao campo
        # Line dist -> verificar se parametrização é em relaçao ao robo ou à alguma das paredes

        # Potential Fields for the base rules of the 
        # right goal area (our team)
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 0.650), # parametrizado em relaçao ao field
                theta = math.pi/2,
                line_size = 0.25, # parametrizado em relaçao ao field
                line_size_max = 0.25, # parametrizado em relaçao ao field
                line_dist = 0.25, # parametrizado em relaçao a area do gol
                line_dist_max = 0.25, # parametrizado em relaçao a area do gol
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

        # Left goal area (opposite team)
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

        #rigth goal area
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (1.5, 0.65), #parametrizar
                theta = math.pi/2,
                line_size = 0.35,
                line_dist = 0.15,
                inverse = True,
                line_dist_max = 0.15,
                line_dist_single_side = True,
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed
            )
        )

        #left goal area
        self.base_rules.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, 0.65), #parametrizar
                theta = math.pi*(3/2),
                line_size = 0.35,
                line_dist = 0.15,
                inverse = True,
                line_dist_max = 0.15,
                line_dist_single_side = True,
                decay = lambda x: x**2,
                multiplier = self.obey_rules_speed
            )
        )

        # Robot avoids other robots
        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue
            self.avoid_obstacles.add_field(
                algorithms.fields.PointField(
                    self.match,
                    target = lambda m, r=robot: (
                        r.x,
                        r.y
                    ),
                    radius = .15,
                    radius_max = .15,
                    decay = lambda x: 1,
                    multiplier = -.5
                )
            )

        self.seek.add_field(self.base_rules)
        self.seek_anti_clockwise.add_field(self.base_rules)

        self.seek_clockwise.add_field(self.base_rules)

        self.tackle.add_field(self.base_rules)
        self.defend.add_field(self.base_rules)
        self.defend.add_field(self.avoid_obstacles)
        self.carry.add_field(self.base_rules)
        self.carry2.add_field(self.base_rules)
        self.kick.add_field(self.base_rules)

        # Potential field for the defend behavior
        self.defend.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.75, 0.65), # center of the field
                radius = 0.2, # 20cm - radius of the center of the field
                decay = lambda x: x**2,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.5
            )
        )

        # Potential field for the kick behavior
        self.kick.add_field(
            algorithms.fields.PointField(
                self.match,
                target = self.goal_aim_y, # point of the line of the goal that the robot is aimed at
                radius = 0.4, # goal width
                decay = lambda x: x**2,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.5
            )
        )

        #Potential field for the tackle behaviour
        self.tackle.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.2, # 30cm
                decay = lambda x: x**2,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.5 # 50 cm/s
            )
        )

        #Potential field for the tackle behaviour
        self.tackle.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0.4, 0.65),
                theta = math.pi/2,
                line_size = 0.2,
                line_dist = 0.2,
                line_dist_max = 0.5,
                decay = lambda x: x,
                multiplier = 0.5,
                line_dist_single_side = True
            )
        )
        
        # Seek behaviour
        self.seek.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m, r=self: r.generate_graph([m.ball.x, m.ball.y]),
                radius = 0.2, # 30cm
                decay = lambda x: x**2,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.8 # 80 cm/s
            )
        )

        # Anti clockwise Seek behaviour
        self.seek_clockwise.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = lambda m: (m.ball.x-0.02, m.ball.y-0.1),
                radius = 0.1, # 6cm
                radius_max = 0.22, # 8cm
                clockwise = False,
                decay = lambda x: 1,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.8 # 100 cm/s
            )
        )

        # Clockwise Seek behaviour
        self.seek_anti_clockwise.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = lambda m: (m.ball.x-0.02, m.ball.y+0.1),
                radius = 0.1, # 6cm
                radius_max = 0.22, # 8cm
                clockwise = True,
                decay = lambda x: 1,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.8 # 100 cm/s
            )
        )

        self.seek_anti_clockwise.add_field(self.seek)
        self.seek_clockwise.add_field(self.seek)

        # Carry behaviour
        self.carry.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m, r=self: r.generate_graph([m.game.field.get_dimensions()[0], m.game.field.get_dimensions()[1]/2]),
                radius = 0.2, # 20cm
                decay = lambda x: 1,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.7 # 70 cm/s
            )
        )

        # Carry behaviour 2
        self.carry2.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m, r=self: r.generate_graph([m.ball.x, m.ball.y]),
                radius = 0.2, # 20cm
                decay = lambda x: 1,
                field_limits = [0.75*2 , 0.65*2],
                multiplier = 0.7 # 70 cm/s
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
        Verificar qual comportamento será usado.
        Caso o behavior use astar, usar astarVoronoi para criar o grafo, passar para o astar
        e pegar o caminho que o robô seguirá (self.path), entao usar path para pegar as
        coordenadas alvo do robo.
        avoid_obstacles usa pontos de repulsão nos robos.
        Ana
        seek, caso a bola não esteja em posse de nenhum time, procurar a bola.
         seek usa astar
         caso nao haja robo a menos de alguns cm de distancia da bola.
        
        Maria
        tackle, caso a bola esteja com o adversario, pegar a bola.
         usar so campo potencial de ponto de atracao na bola
         caso robo adversario esta perto da bola e robos aliados nao, dentro de alguns cm.
	Mat
        defend, caso a bola esteja com outro robo aliado, ficar de guarda.
         so campos potencias
         tem como saber o behavior atual de outro robo (meio campista por ex)?
         ou caso o robo aliado esteja a uma certa distancia da bola sem adversario mais perto.
        
        se o atacante tiver a bola usamos astar:
	
	Ana
        kick, caso o atacante esteja pronto para chutar ao gol.
         so campo potencial ate o ponto do gol no qual o atacante esta mirando (entre y>=0.49 e y<=0.81)
         ou se o path so tem pos atual do robo e a proxima pos é a do gol.
	Mat
        carry, caso o atacante esteja com a bola, levar até o gol.
         astar
         usar path pra saber proximo ponto para o qual o robo leva a bola.
        """
        
        # 0 - posse do atacante, 1 - nosso time, 2 - outro time, 3 - posse de ninguem
        
        all_robots = self.match.robots + self.match.opposites
        possession = self.get_ball_possession(all_robots, self.match)
        print(possession)
        
        # goal_aim() -> function to determine if attacker is aiming the ball to the goal
        
        # if possession == 3:
        #   for r in all_robots:
        #       if (r.get_name() != self.robot.get_name()) and (r.team_color != self.robot.team_color):
        #           if(dist_to_ball(r, m) <= 24 * 10**-2):
        # 	            behaviour = self.tackle
        # 	 behaviour = self.seek
        # elif possession == 2:
        # 	behaviour = self.tackle
        # elif possession == 1:
        # 	behaviour == self.defend
        # if goal_aim() == False:
        #  	behaviour = self.carry
        #     voronoi = new Scratch(self.match, [OPPOSITE_GOAL_X, fd.get_dimensions[1]/2])
        #     vf = behaviour.compute([self.robot.x, self.robot.y])
        #     vf = (vf[0] + Scratch[1][0], vf[1] + self.voronoi[1][1])
        #     return vf
        # else:
        #     behaviour = self.kick

        # 0 - posse do atacante, 1 - nosso time, 2 - outro time, 3 - posse de ninguem
        if possession == 0 and math.dist((self.robot.x, self.robot.y), (1.500, 0.650)) <= 0.4 and self.goal_aim(self.robot) == True:
            behaviour = self.kick
        elif possession == 0: # and self.match.ball.x > self.robot.x:
            if(self.ball_control == True):
                behaviour = self.carry2
            else:
                behaviour = self.carry
        elif possession == 1:
            behaviour = self.defend
        elif possession == 2:
            behaviour = self.tackle
        else:
            for r in all_robots:
                if (self.dist_to_ball(r, self.match) <= 24 * 10**-2):
                    if (r.get_name() != self.robot.get_name()) and (r.team_color != self.robot.team_color):
                        break
        	        # behaviour = self.tackle
                else:
                    if self.robot.x > self.match.ball.x-0.23 and self.dist_to_ball(self.robot, self.match) <= 0.2:
                        if self.robot.y < self.match.ball.y:
                            behaviour = self.seek_clockwise
                        else:
                            behaviour = self.seek_anti_clockwise
                    else:
                        behaviour = self.seek
        
        print(behaviour.name)
        #return super().decide(behaviour)
        return behaviour.compute([self.robot.x, self.robot.y])