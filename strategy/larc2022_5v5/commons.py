import math
from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.potential_fields import fields
from controller.simple_LQR import TwoSidesLQR
from strategy.utils.player_playbook import PlayerPlay

def aim_projection_ball(strategy):
    m = strategy.match
    b = strategy.match.ball
    r = strategy.robot

    ball = [b.x, b.y]
    goal_pos = [
        m.game.field.get_dimensions()[0],
        m.game.field.get_dimensions()[1]/2
    ]

    dir_to_goal_vector = [
        goal_pos[0] - ball[0], 
        goal_pos[1] - ball[1]
    ]
    angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

    side = -1 if b.y > r.y else 1

    side_size = 0.05
    recoil_size = 0.05

    angle_of_attack = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0]) + math.pi/2

    point_with_recoil = [ball[0] - recoil_size* math.cos(angle), ball[1] - recoil_size * math.sin(angle)]

    point_of_attack = [
        point_with_recoil[0] + side * side_size * math.cos(angle_of_attack), 
        point_with_recoil[1] + side * side_size * math.cos(angle_of_attack)
    ]

    return point_of_attack

class AstarPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(
            match, 
            robot
        )

        self.actual_iteration = 0
        self.next_iterarion = 0

        self.path = []
        self.next_path = []

    def start_up(self):
        super().start_up()
        controller = TwoSidesLQR
        controller_kwargs = {}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        self.astar = fields.PotentialField(
            self.match,
            name="{}|AstarBehaviour".format(self.__class__)
        )

        def astar_eval(m, s=self):
            res = voronoi_astar(
                    s.robot.strategy, s.match, aim_projection_ball
                )
            if len(res) > 1:
                return res[1]
            else:
                return [0, 0]

        self.astar.add_field(
            fields.PointField(
                self.match,
                target = astar_eval,
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

    def get_name(self):
        return f"<{self.robot.get_name()} Astar Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        res = self.astar.compute(robot_pos)
        
        return res