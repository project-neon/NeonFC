import math

import numpy as np
import numpy.linalg as la

from scipy.spatial import Voronoi

from controller import TwoSidesLQR

from algorithms.astar.astar import AStar, Node
from algorithms.astar.fieldGraph import FieldGraph
from algorithms.potential_fields.fields import PointField, LineField, TangentialField, PotentialField

from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

def distance_to_line(x, y, l1x, l1y, l2x, l2y):
    x_diff = l2x - l1x
    y_diff = l2y - l1y
    num = abs(y_diff*x - x_diff*y + l2x*l1y - l2y*l1x)
    den = math.sqrt(y_diff**2 + x_diff**2)
    return num / den

# class OffensivePlay(DebugPotentialFieldStrategy):
class OffensivePlay(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game

        self.graph = FieldGraph()
        self.ctrl_params = {"l": 0.115}

        super().__init__(
            match, 'DefensiveVoronoi',
            controller=TwoSidesLQR,
            controller_kwargs=self.ctrl_params
        )

    def start(self, robot=None):
        super().start(robot=robot)
        
        self.tangential = None

        self.point = PotentialField(self.match, name="astar")

        self.dribble = PotentialField(self.match, name="dribble")

        self.keep = PotentialField(self.match, name="keep")

        self.push = PotentialField(self.match, name="push")


        self.point.add_field(
            PointField(
                self.match,
                target = lambda m, r=self : r.voronoi_astar(m),
                radius = 0.1,
                decay = lambda x: x,
                multiplier = 0.8
            )
        )

        self.push.add_field(
            PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.05, # 30cm
                decay = lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = lambda m: max(0.80, math.sqrt(m.ball.vx**2 + m.ball.vy**2) + 0.25) # 50 cm/s
            )
        )

        self.push.add_field(
            LineField(
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
                multiplier = 0.7 # 75 cm/s
            )
        )


        self.keep.add_field(
            LineField(
                self.match,
                target= lambda m: (
                    m.ball.x - math.cos(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025, 
                    m.ball.y - math.sin(math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))) * 0.025
                ),
                theta=lambda m: ( -math.atan2((0.65-m.ball.y), (0.75*2 - m.ball.x))),
                line_size = .1,
                line_size_single_side = True,
                line_dist = .2,
                line_dist_max = .2,
                decay = lambda x: x,
                multiplier = .5
            )
        )

        self.point.add_field(self.dribble)

        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue        
            self.dribble.add_field(
                    PointField(
                        self.match,
                        target = lambda m, r=robot: (
                            r.x,
                            r.y
                        ),
                        radius = .2,
                        radius_max = .2,
                        decay = lambda x: -x +1,
                        multiplier = -.75
                    )
                )
    
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def voronoi_astar(self, m):
        self.graph = FieldGraph()

        self.robot_node = Node([self.robot.x, self.robot.y])
        self.graph.set_start(self.robot_node)

        self.robot_node.position = [self.robot.x, self.robot.y]

        corners = [
            [self.match.game.field.get_dimensions()[0], 0],
            [0, 0],
            [0, self.match.game.field.get_dimensions()[1]],
            [self.match.game.field.get_dimensions()[0], self.match.game.field.get_dimensions()[1]],

            [0, self.match.game.field.get_dimensions()[1]/2],
            [self.match.game.field.get_dimensions()[0], self.match.game.field.get_dimensions()[1]/2],

            [0, self.match.game.field.get_dimensions()[1]/4],
            [self.match.game.field.get_dimensions()[0], self.match.game.field.get_dimensions()[1]/4],

            [0, 3 * self.match.game.field.get_dimensions()[1]/4],
            [self.match.game.field.get_dimensions()[0], 3 * self.match.game.field.get_dimensions()[1]/4],
        ]
        mergeble_obstacles = [ [r.x, r.y] for r in self.match.opposites] + [
            [r.x, r.y] for r in self.match.robots 
            if r.robot_id != self.robot.robot_id
        ]

        dist = lambda x, y: math.sqrt( (x[0] - y[0])**2 + (x[1] - y[1])**2 )
        to_merge = []
        to_remove = []
        for i1, m1 in enumerate(mergeble_obstacles):
            for i2, m2 in enumerate(mergeble_obstacles):
                if m1 == m2: continue
                if dist(m1, m2) <= 2 * self.robot.dimensions["L"] * math.sqrt(2):
                    to_merge.append([m1, m2])
                    to_remove.append(i1)
                    to_remove.append(i2)
        
        to_remove = list(set(to_remove))
        for index in sorted(to_remove, reverse=True):
            del mergeble_obstacles[index]

        for o1, o2 in to_merge:
            mergeble_obstacles.append([ (o1[0] + o2[0])/2 , (o1[1] + o2[1])/2 ])

        unmergeble_obsctacles = [ [self.match.ball.x, self.match.ball.y], self.robot_node.position]


        obstacles = corners + mergeble_obstacles + unmergeble_obsctacles

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

        # return [
        #     speed * (path[1][0] - path[0][0])/dist,
        #     speed * (path[1][1] - path[0][1])/dist
        # ]
        return  path[1]
    
    def get_range_of_aim(self):
        goal_ = self.match.game.field.get_dimensions()
        goal_left = [goal_[0], goal_[1]/2 + 0.15]
        goal_right = [goal_[0], goal_[1]/2 - 0.15]
        ball_pos = [self.match.ball.x, self.match.ball.y]

        range_ = []

        for goal in [goal_left, goal_right]:
            ball_to_goal_or = math.atan2(ball_pos[0] - goal[0], ball_pos[1] - goal[1])

            robot_to_goal_or = math.atan2(
                self.robot.x - goal[0], self.robot.y - goal[1]
            )

            ball_to_goal_or = math.degrees(ball_to_goal_or)
            if ball_to_goal_or < 0:
                ball_to_goal_or += 90

            robot_to_goal_or = math.degrees(robot_to_goal_or)
            if robot_to_goal_or < 0:
                robot_to_goal_or += 90

            print(f"#### {goal}: {ball_to_goal_or} {robot_to_goal_or}")
            
            range_.append((robot_to_goal_or - ball_to_goal_or))

        print(f"RANGE {range_}")
        return range_


        


    def has_possession(self, ball_ahead=True):
        if ball_ahead and self.match.ball.x < self.robot.x:
            return False
        
        goal = self.match.game.field.get_dimensions()
        goal = [goal[0], goal[1]/2]

        ranges = self.get_range_of_aim()

        distance_from_aim = distance_to_line(
            self.match.ball.x, self.match.ball.y,
            math.sin(self.robot.theta), math.cos(self.robot.theta),
            self.robot.x, self.robot.y
        )

        dist_robot_ball = (
            (self.robot.x - self.match.ball.x)**2
            + (self.robot.y - self.match.ball.y)**2
        )**.5

        # print(f"dist from aim {distance_from_aim}")
        # print(f"dist to ball {dist_robot_ball}")

        if (
            distance_from_aim < 0.075
            and ranges[0] > 0 and ranges[1] < 0
            and dist_robot_ball < 0.15
        ):
            return True
        return False

    def decide(self):
        ball = self.match.ball
        robot = self.robot


        dist_to_ball = ( (ball.x - robot.x)**2 +  (ball.y - robot.y)**2 )**.5

        field_limits = self.match.game.field.get_dimensions()
        mid_field = [ax/2 for ax in field_limits]

        aim_param = 0.3 * max(0, mid_field[0] - robot.x)

        mid_field[0] += aim_param

        tangential_radius = 0.3

        def choose_target(m, r_id):
            def f(m):
                _sign = 1 if choose_ccw(m, r_id) else -1
                return (
                    m.ball.x + _sign * math.cos(math.atan2((mid_field[1] - m.ball.y), (mid_field[0]*2 - m.ball.x)) + math.pi/2) * 0.4 * dist_to_ball,
                    m.ball.y + _sign * math.sin(math.atan2((mid_field[1] - m.ball.y), (mid_field[0]*2 - m.ball.x)) + math.pi/2) * 0.4 * dist_to_ball
                )

            return f
        
        def choose_ccw(m, r_id):
            above_line = lambda A, B, X, Y: ((B[0] - A[0]) * (Y - A[1]) - (B[1] - A[1]) * (X - A[0]))
            fl = field_limits
            _sign = above_line([m.ball.x, m.ball.y], [fl[0], fl[1]/2], m.robots[r_id].x, m.robots[r_id].y)
            if abs(_sign) <= 0.05:
                return True
            return _sign > 0
            

        behaviour = None

        if self.has_possession():
            behaviour = self.push
        elif dist_to_ball >= tangential_radius:
            self.tangential = None
            
            behaviour = self.point
        else:
            if self.tangential:
                behaviour = self.tangential
            else:
                self.tangential = PotentialField(self.match)
                self.tangential.add_field(self.dribble)
                self.tangential.add_field(self.keep)

                self.tangential.add_field(
                    PointField(
                        self.match,
                        target = lambda m: (
                            m.ball.x,
                            m.ball.y
                        ),
                        radius = .35,
                        radius_max = .35,
                        decay = lambda x: (1 - x) ** 4,
                        multiplier = -.5
                    )
                )

                self.tangential.add_field(
                    TangentialField(
                        self.match,
                        target=choose_target(self.match, self.robot.robot_id),                                                                                                                                                                                                                                                                                                                                          
                        radius = dist_to_ball * 0.2,
                        radius_max = dist_to_ball * 10,
                        clockwise = lambda m, r_id=self.robot.robot_id: choose_ccw(m, r_id),
                        decay=lambda _: 1,
                        multiplier = lambda m, me=self.robot: min(
                            max(0.75,  m.ball.get_speed() + .2 ), 0.85)
                    )
                )

                behaviour = self.tangential
        
        print(behaviour.name)
        # return super().decide(behaviour)
        return behaviour.compute([self.robot.x, self.robot.y])
        
        
