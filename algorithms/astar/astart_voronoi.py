import sys
import math
import copy
from algorithms.astar.astar import AStar, Node, distance
from algorithms.astar.fieldGraph import FieldGraph

from scipy.spatial import Voronoi



def sample_astar(strategy, objective_function, graph):
    g = copy.copy(graph)

    robot_node = Node([strategy.robot.x, strategy.robot.y])

    objective_node = Node(objective_function(strategy))

    g.add_node(robot_node)
    g.add_node(objective_node)

    g.set_start(robot_node)

    for node in g.nodes:
        if distance(node.position, robot_node.position) <= 0.05:
            g.add_edge([robot_node, node])

    for node in g.nodes:
        if distance(node.position, objective_node.position) <= 0.05:
            g.add_edge([objective_node, node])

    return AStar(robot_node, objective_node).calculate()

    



    




def voronoi_astar(strategy, m, objective_function):
        strategy.graph = FieldGraph()

        strategy.robot_node = Node([strategy.robot.x, strategy.robot.y])
        strategy.graph.set_start(strategy.robot_node)

        strategy.robot_node.position = [strategy.robot.x, strategy.robot.y]

        corners = [
            [strategy.match.game.field.get_dimensions()[0], 0],
            [0, 0],
            [0, strategy.match.game.field.get_dimensions()[1]],
            [strategy.match.game.field.get_dimensions()[0], strategy.match.game.field.get_dimensions()[1]],

            [0, strategy.match.game.field.get_dimensions()[1]/2],
            [strategy.match.game.field.get_dimensions()[0], strategy.match.game.field.get_dimensions()[1]/2],

            [0, strategy.match.game.field.get_dimensions()[1]/4],
            [strategy.match.game.field.get_dimensions()[0], strategy.match.game.field.get_dimensions()[1]/4],

            [0, 3 * strategy.match.game.field.get_dimensions()[1]/4],
            [strategy.match.game.field.get_dimensions()[0], 3 * strategy.match.game.field.get_dimensions()[1]/4],
        ]
        mergeble_obstacles = [ [r.x, r.y] for r in strategy.match.opposites] + [
            [r.x, r.y] for r in strategy.match.robots 
            if r.robot_id != strategy.robot.robot_id
        ] + [[strategy.match.ball.x, strategy.match.ball.y]]

        dist = lambda x, y: math.sqrt( (x[0] - y[0])**2 + (x[1] - y[1])**2 )
        to_merge = []
        to_remove = []
        for i1, m1 in enumerate(mergeble_obstacles):
            for i2, m2 in enumerate(mergeble_obstacles):
                if m1 == m2: continue
                if dist(m1, m2) <= 2 * strategy.robot.dimensions["L"] * math.sqrt(2):
                    to_merge.append([m1, m2])
                    to_remove.append(i1)
                    to_remove.append(i2)
        
        to_remove = list(set(to_remove))
        for index in sorted(to_remove, reverse=True):
            del mergeble_obstacles[index]

        for o1, o2 in to_merge:
            mergeble_obstacles.append([ (o1[0] + o2[0])/2 , (o1[1] + o2[1])/2 ])

        
        objective =  objective_function(strategy) #strategy.match.ball

        unmergeble_obsctacles = [ objective, strategy.robot_node.position]


        obstacles = corners + mergeble_obstacles + unmergeble_obsctacles

        vor = Voronoi(obstacles)

       
        target_node = Node(objective)

        nodes = [
            Node([a[0], a[1]]) for a in vor.vertices
        ] + [
            target_node, strategy.robot_node
        ]

        objective_index = len(obstacles) - 2
        robot_index = len(obstacles) - 1
        
        strategy.graph.set_nodes(nodes)

        polygon_objective_edges = []
        polygon_robot_edges = []

        for edge, ridge_vertice in zip(vor.ridge_vertices, vor.ridge_points):
            if edge[0] == -1: continue
            strategy.graph.add_edge([nodes[edge[0]], nodes[edge[1]]])

            if objective_index in ridge_vertice:
                polygon_objective_edges.append(nodes[edge[0]])
                polygon_objective_edges.append(nodes[edge[1]])

            if robot_index in ridge_vertice:
                polygon_robot_edges.append(nodes[edge[0]])
                polygon_robot_edges.append(nodes[edge[1]])
            
            if objective_index in ridge_vertice and robot_index in ridge_vertice:
                strategy.graph.add_edge([strategy.robot_node, target_node])

        for edge_to_ball in set(polygon_objective_edges):
            strategy.graph.add_edge([edge_to_ball, target_node])

        for edge_to_ball in set(polygon_robot_edges):
            strategy.graph.add_edge([edge_to_ball, strategy.robot_node])


        return AStar(strategy.robot_node, target_node).calculate()