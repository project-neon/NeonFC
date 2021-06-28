import math
import random

from scipy.spatial import Voronoi

from algorithms.astar.astar import AStar, Node
from algorithms.astar.fieldGraph import FieldGraph
from algorithms.potential_fields.fields import TangentialField, PotentialField
from strategy.BaseStrategy import Strategy

class OffensivePlay(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game

        self.graph = FieldGraph()

        super().__init__(match, 'DefensiveVoronoi')

    def start(self, robot=None):
        super().start(robot=robot)
        self.tangential = None
    
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def voronoi_astar(self, speed):
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
        ] + [ [self.match.ball.x, self.match.ball.y], self.robot_node.position]

        vor = Voronoi(obstacles)

        objective = self.match.ball
        target_node = Node([objective.x, objective.y])

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

        for edge_to_ball in set(polygon_objective_edges):
            self.graph.add_edge([edge_to_ball, target_node])

        for edge_to_ball in set(polygon_robot_edges):
            self.graph.add_edge([edge_to_ball, self.robot_node])


        path = AStar(self.robot_node, target_node).calculate()

        dist = ((path[0][0] - path[1][0])**2 + (path[1][1] - path[1][1])**2 )**.5

        return [
            speed * (path[1][0] - path[0][0])/dist,
            speed * (path[1][1] - path[0][1])/dist
        ]


    def decide(self):
        ball = self.match.ball
        robot = self.robot

        dist_to_ball = ( (ball.x - robot.x)**2 +  (ball.y - robot.y)**2 )**.5

        field_limits = self.match.game.field.get_dimensions()
        mid_field = [ax/2 for ax in field_limits]
        robot_speed = ( (robot.vx)**2 + (robot.vy)**2 )**.5

        tangential_radius = 0.25

        def choose_target(m, r_id):
            above_line = lambda A, B, X, Y: 1 if ((B[0] - A[0]) * (Y - A[1]) - (B[1] - A[1]) * (X - A[0])) > 0 else -1
            fl = field_limits
            
            def f(m):
                _sign = 1 #above_line([m.ball.x, m.ball.y], [fl[0], fl[1]/2], m.robots[r_id].x, m.robots[r_id].y)
                return (
                    m.ball.x + _sign * math.cos(math.atan2((mid_field[1] - m.ball.y), (mid_field[0]*2 - m.ball.x)) + math.pi/2) * 0.4 * dist_to_ball,
                    m.ball.y + _sign * math.sin(math.atan2((mid_field[1] - m.ball.y), (mid_field[0]*2 - m.ball.x)) + math.pi/2) * 0.4 * dist_to_ball
                )

            return f
        
        def choose_ccw(m, r_id):
            above_line = lambda A, B, X, Y: 1 if ((B[0] - A[0]) * (Y - A[1]) - (B[1] - A[1]) * (X - A[0])) > 0 else -1
            fl = field_limits

            def f(m):
                _sign = above_line([m.ball.x, m.ball.y], [fl[0], fl[1]/2], m.robots[r_id].x, m.robots[r_id].y)
                return -_sign
            
            return f

        if dist_to_ball >= tangential_radius:
            self.tangential = None
            return self.voronoi_astar( max(.2, min(robot_speed * 2.3, .7)) )
        else:
            if self.tangential:
                return self.tangential.compute([self.robot.x, self.robot.y])
            else:
                self.tangential = PotentialField(self.match)

                self.tangential.add_field(
                    TangentialField(
                        self.match,
                        target=choose_target(self.match, self.robot.robot_id),                                                                                                                                                                                                                                                                                                                                          
                        radius = dist_to_ball * 0.2,
                        radius_max = dist_to_ball * 10,
                        clockwise = 1,
                        decay=lambda _: 1,
                        multiplier = 0.75
                    )
                )

                return self.tangential.compute([self.robot.x, self.robot.y])
        
        
