import time

from algorithms.astar import AStar, Node, FieldGraph
from strategy.BaseStrategy import Strategy
from scipy.spatial import Voronoi

class Scratch(Strategy):
    def __init__(self, match):
        super().__init__(match, "AstarVoronoi")

        self.graph = FieldGraph()
        self.min_range = 0.56
        self.robot_node = None


    def start(self, robot=None):
        super().start(robot=robot)


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):
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

        dist = ( (path[0][0] - path[1][0])**2 + (path[1][1] - path[1][1])**2 )**.5

        return [
            0.5 * (path[1][0] - path[0][0])/dist,
            0.5 * (path[1][1] - path[0][1])/dist
        ]

