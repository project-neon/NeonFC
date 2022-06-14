from algorithms.astar import AStar, Node
from scipy.spatial import Voronoi
import algorithms

class PathAstar:
    def __init__(self, match):
        self.graph = algorithms.astar.FieldGraph()
        self.match = match
        self.road = []
        self.corners = [
            [self.match.game.field.get_dimensions()[0], 0],
            [0, 0],
            [0, self.match.game.field.get_dimensions()[1]],
            [self.match.game.field.get_dimensions()[0], self.match.game.field.get_dimensions()[1]]
        ]
    
    def voronoi_graph(self, start = [], target = [], obstacles = []):
        """
        Uses Voronoi in order to create a graph to be used by the astar.
        Returns an array of positions of the nodes of the graph that leads to the target.

        Receives the starting position [x, y]
        Receives the target position [x, y]
        Receives a list of positions of the obstacles [[x1, y1], [x2, y2]]
        """
        self.graph = algorithms.astar.FieldGraph()

        start_node = Node(start)
        self.graph.set_start(start_node)

        target_node = Node(target)
        self.graph.set_start(target_node)

        obstacle_list = self.corners + obstacles + [target] + [start]

        vor = Voronoi(obstacle_list)

        nodes = [
            Node([a[0], a[1]]) for a in vor.vertices
        ] + [
            target_node, start_node
        ]

        objective_index = len(obstacle_list) - 2
        start_index = len(obstacle_list) - 1

        self.graph.set_nodes(nodes)

        polygon_objective_edges = []
        polygon_start_edges = []

        for edge, ridge_vertice in zip(vor.ridge_vertices, vor.ridge_points):
            if edge[0] == -1: continue
            self.graph.add_edge([nodes[edge[0]], nodes[edge[1]]])

            if objective_index in ridge_vertice:
                polygon_objective_edges.append(nodes[edge[0]])
                polygon_objective_edges.append(nodes[edge[1]])

            if start_index in ridge_vertice:
                polygon_start_edges.append(nodes[edge[0]])
                polygon_start_edges.append(nodes[edge[1]])
            
            if objective_index in ridge_vertice and start_index in ridge_vertice:
                self.graph.add_edge([start_node, target_node])

        for edge_to_target in set(polygon_objective_edges):
            self.graph.add_edge([edge_to_target, target_node])

        for edge_to_start in set(polygon_start_edges):
            self.graph.add_edge([edge_to_start, start_node])

        road = AStar(start_node, target_node).calculate()

        return road

    def calculate(self, start = [], target = [], obstacles = []):
        self.road = self.voronoi_graph(start, target, obstacles)
        dist = ( (self.road[0][0] - self.road[1][0])**2 + (self.road[0][1] - self.road[1][1])**2 )**.5
        r_v = [
            0.5 * (self.road[1][0] - self.road[0][0])/dist,
            0.5 * (self.road[1][1] - self.road[0][1])/dist
        ]
        return r_v
