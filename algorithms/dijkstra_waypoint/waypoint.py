from algorithms.dijkstra_waypoint import dijkstra
from commons.math import dist_point_line

class WaypointSystem():
    def __init__(self,
        min_range=0.30, max_range=1.50,
        robot_radius=0.10,wall_anchors = 4
        ):

        self.min_range = min_range
        self.max_range = max_range
        self.robot_radius = robot_radius

        self.dijkstra = None
        self.graph = None

    def start(self):
        pass

    def update(self, me, objective=None, obstacles=[], field_size = [200, 180]):
        self.graph = dijkstra.Graph()

        self.graph.add_vertex(f'{objective.get_name()}', objective.x, objective.y)
        self.graph.add_vertex(f'{me.get_name()}', me.x, me.y)

        for obs1 in obstacles:
            for obs2 in obstacles:
                if obs1.get_name() == obs2.get_name():
                    continue
                x = (obs1.x + obs2.x)/2
                y = (obs1.y + obs2.y)/2

                dist = ( (obs1.x - obs2.x)**2 + (obs1.y - obs2.y)**2 )**(1/2)

                if dist > self.min_range and not self.graph.vert_dict.get(f'{obs2.get_name()}_{obs1.get_name()}'):
                    self.graph.add_vertex(f'{obs1.get_name()}_{obs2.get_name()}', x, y)

        for vtx1 in self.graph.vert_dict.values():
            for vtx2 in self.graph.vert_dict.values():
                if (vtx1.get_id() == vtx2.get_id()) or (vtx1.x == vtx2.x and vtx1.y == vtx2.y):
                    continue
                if any(
                        [
                            dist_point_line(vtx1.x, vtx1.y, vtx2.x, vtx2.y, o.x, o.y) < self.robot_radius
                            for o in obstacles
                        ]
                    ):
                    continue
                self.graph.add_edge(vtx1, vtx2)
    
    def dir_vector(self, path, robot):
        target = self.graph.vert_dict[path[1]] if len(path) > 1 else self.graph.vert_dict[path[0]]
        dist = ( (target.x - robot.x)**2 + (target.y - robot.y)**2 )**.5
        return [
            (target.x - robot.x)/dist,
            (target.y - robot.y)/dist
        ]

    def decide(self, robot, target, speed):
        robot_name = robot.get_name()
        target_name = target.get_name()

        dijkstra.dijkstra(self.graph, robot_name, target_name)

        target = self.graph.get_vertex(target_name)

        path = [target.get_id()]

        dijkstra.shortest(target, path)

        return [v * speed for v in self.dir_vector(path[::-1], robot)]
