import math
from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.limit_cycle.limit_cycle import Obstacle, Point
from algorithms.potential_fields import fields
from algorithms.limit_cycle import LimitCycle
from controller.PID_control import PID_control
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

        self.astar.add_field(
            fields.LineField(
                self.match,
                target= [self.match.game.field.get_dimensions()[0] - self.match.game.field.get_dimensions()[0], 
                self.match.game.field.get_dimensions()[1]/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi/2,
                line_size = (self.match.game.field.get_small_area("defensive")[3]/2),
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: 1,
                multiplier = -2
            )
        )

    def get_name(self):
        return f"<{self.robot.get_name()} Astar Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        res = self.astar.compute(robot_pos)

        return res


class LimitCyclePlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(
            match, 
            robot
        )

        self.limit_cycle = LimitCycle(self, True)
        self.BALL_Y_MAX = 1.25
        self.BALL_Y_MIN = 0.25

        self.dl = 0.000001
        self.shooting_momentum = 0

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {'max_speed': 2.5, 'max_angular': 2400}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

        self.shooting_momentum = 0

    def update_momentum(self):
        angle_between = lambda p1, p2 : math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        dist = lambda p1, p2: ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**.5

        theta = self.robot.theta
        x = self.robot.x
        y = self.robot.y
        ball = [self.match.ball.x, self.match.ball.y]
        goal = [1.5, .65]

        # diference between robot angle and (ball-robot) angle
        c1v = abs(theta - angle_between([x, y], ball))
        c1 = c1v <= .4 or abs(c1v - math.pi) <= .4

        # diference between robot angle and (goal-robot) angle
        c2v = abs(theta - angle_between([x, y], goal)) #<= .5
        c2 = c2v <= .4 or abs(c2v - math.pi) <= .4

        # distance between ball and robot
        c3 = dist([x, y], ball) <= .25

        # if c3:
        #     print(c1, c2, c3)

        if c1 and c2 and c3:
            self.shooting_momentum = 100 * dist([x, y], goal)

    def start(self):
        pass

    def get_name(self):
        return f"<{self.robot.get_name()} Limit Cycle Planning>"

    def update(self):

        return self.decide()

    def decide(self):
        x = self.robot.x
        y = self.robot.y

        ball_virtual_y = max(self.BALL_Y_MIN, min(self.BALL_Y_MAX, self.match.ball.y))

        robot = Point(x, y)
        target = Point(self.match.ball.x, ball_virtual_y)

        if not (0 <= target.x <= 1.5) and not (0 <= target.y <= 1.3):
            target = Point(self.limit_cycle.target.x, self.limit_cycle.target.y)

        boundaries = [
            Obstacle(r.x, r.y, 0.075) for r in self.match.robots + self.match.opposites if r.get_name() != self.robot.get_name()
        ]

        boundaries += [
            Obstacle(self.robot.x, 0, 0.01),
            Obstacle(self.robot.x, self.match.game.field.get_dimensions()[1], 0.01),
            Obstacle(self.match.game.field.get_dimensions()[0], self.robot.y, 0.01),
            Obstacle(self.match.game.field.get_dimensions()[0], 0, 0.01),
        ]

        boundaries += [
            Obstacle(0, self.match.game.field.get_dimensions()[1]/2, 0.40),
        ]

        self.limit_cycle.update(robot, target, [*boundaries])
        desired = self.limit_cycle.compute()

        self.update_momentum()
        if self.shooting_momentum > 0:
            self.control_linear_speed = False
            desired = [1.5, .65]
            self.shooting_momentum -= 1
        else:
            self.control_linear_speed = True
            self.lp = [self.match.ball.x, self.match.ball.y]

        return desired