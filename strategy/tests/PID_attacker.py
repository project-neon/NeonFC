from controller.simple_LQR import SimpleLQR
import math
import algorithms
from commons import math as nfc_math
from strategy.BaseStrategy import Strategy
from controller import PID_control
from strategy.DebugTools import DebugPotentialFieldStrategy
from algorithms.astar import AStar, Node
from algorithms.astar.fieldGraph import FieldGraph
from scipy.spatial import Voronoi


#class Attacker(DebugPotentialFieldStrategy):
class Attacker(Strategy):

    def __init__(self, match, name = 'PID-Attacker'):
        #self.ctrl_params = {"l": 0.07}
        super().__init__(match,
            name=name,
            controller=PID_control,
            #controller_kwargs=self.ctrl_params
        )

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

        if len(path) < 2:
            return [0, 0]

        dist = ((path[0][0] - path[1][0])**2 + (path[1][1] - path[1][1])**2 )**.5

        return  path[1]

    def start(self, robot=None):
        super().start(robot=robot)

        uvf_radius = 0.075 # 7.5 cm
        uvf_radius_2 = 0.075 # 7.5 cm
    
        tangential_speed = lambda m: max(.8, (m.ball.vx**2 + m.ball.vy**2)**.5 + .3 ) # 8 cm/s

        """
        MTG-UVF: move to goal univector field

        2 campos tangenciais
        target: deslocamento para o lado da bola em relacao ao gol
        multiplier: media ponderada em relacao ao raio
        """

        self.seek = algorithms.fields.PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        self.wait = algorithms.fields.PotentialField(
            self.match,
            name="{}|WaitBehaviour".format(self.__class__)
        )

        self.avoid_area = algorithms.fields.PotentialField(
            self.match,
            name="{}|AvoidAreaBehaviour".format(self.__class__)
        )

        self.astar = algorithms.fields.PotentialField(
            self.match,
            name="{}|AstarBehaviour".format(self.__class__)
        )

        def get_aim_point(m):
            field_w, field_h = m.game.field.get_dimensions()
            aim_point_x = field_w
            if field_h/2 - 0.2 < m.ball.y < field_h/2 + 0.2:
                aim_point_y = m.ball.y
            else:
                aim_point_y = field_h/2
            return aim_point_x, aim_point_y

        def shifted_target_left(m, radius_2=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)

            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05

            pos_x = (
                m.ball.x -
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2)*radius_2
            )

            pos_y = (
                m.ball.y -
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2)*radius_2
            )

            return [pos_x, pos_y]

        def shifted_target_right(m, radius=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)

            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05

            pos_x = (
                m.ball.x +
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2) * radius
            )

            pos_y = (
                m.ball.y +
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2) * radius
            )

            return [pos_x, pos_y]
        
        def uvf_mean_contributtion_left(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)

            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05
            
            target = [
                m.ball.x,
                m.ball.y
            ]

            target2 = [
                m.ball.x +
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))) * 0.025,
                m.ball.y +
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x- m.ball.x))) * 0.025
            ]

            dist = nfc_math.distance_to_line(pos[0], pos[1], target[0], target[1], target2[0], target2[1])
            if abs(dist) > radius and dist < 0:
                return 0
            dist = 0.5 * max(0, min(dist, radius))/ (radius)
            return speed(m) * (dist + 0.5)

        def uvf_mean_contributtion_right(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)
            
            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05
            
            target = [
                m.ball.x,
                m.ball.y
            ]

            target2 = [
                m.ball.x +
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))) * 0.025,
                m.ball.y +
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x- m.ball.x))) * 0.025
            ]

            dist = -nfc_math.distance_to_line(pos[0], pos[1], target[0], target[1], target2[0], target2[1])
            if abs(dist) > radius and dist < 0:
                return 0
            dist = 0.5 * max(0, min(dist, radius))/ (radius )
            return speed(m) * (dist + 0.5)

        self.seek.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=shifted_target_left,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_left
            )
        )

        self.seek.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=shifted_target_right,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_right
            )
        )

        field = self.match.game.field.get_dimensions()
        self.wait.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (field[0]/3, field[1]/2),
                radius = 0.1,
                decay = lambda x: x,
                multiplier = 0.60
            )
        )

        # Avoid defensive area
        self.avoid_area.add_field(
            algorithms.fields.LineField(
                self.match,
                target= [self.match.game.field.get_dimensions()[0] - self.match.game.field.get_dimensions()[0], 
                self.match.game.field.get_dimensions()[1]/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi/2,
                line_size = (self.match.game.field.get_small_area("defensive")[3]/2) + 0.08,
                line_dist = 0.23,
                line_dist_max = 0.23,
                decay = lambda x: 1,
                multiplier = -2
            )
        )

        self.wait.add_field(self.avoid_area)
        self.seek.add_field(self.avoid_area)

        self.astar.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m, r=self : r.voronoi_astar(m),
                radius = .075,
                decay = lambda x: x,
                multiplier = tangential_speed
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):
        small_area = self.match.game.field.get_small_area("defensive")
        goal_area = [
            0             - self.robot.dimensions["L"]/2, 
            small_area[1] - self.robot.dimensions["L"]/2, 
            small_area[2] + self.robot.dimensions["L"]/2, 
            small_area[3] + self.robot.dimensions["L"]/2
        ]
        ball = [self.match.ball.x, self.match.ball.y]
        
        dist_to_ball = ( (ball[0] - self.robot.x)**2 +  (ball[1] - self.robot.y)**2 )**.5
        tangential_radius = 0.3

        if nfc_math.point_in_rect(ball, goal_area):
            behaviour = self.wait
        elif dist_to_ball >= tangential_radius:
            behaviour = self.astar
        else:
            behaviour = self.seek

        return behaviour.compute([self.robot.x, self.robot.y])
        #return super().decide(behaviour)
