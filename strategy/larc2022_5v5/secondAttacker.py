import threading
from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.potential_fields import fields
from controller.PID_control import PID_control
from strategy.BaseStrategy import Strategy
import algorithms
import math

from strategy.utils.player_playbook import OnNextTo, PlayerPlay, PlayerPlaybook

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
        controller = PID_control
        controller_kwargs = {'max_speed': 2.8, 'max_angular': 4800}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        self.astar = fields.PotentialField(
            self.match,
            name="{}|AstarBehaviour".format(self.__class__)
        )

        self.astar.add_field(
            fields.PointField(
                self.match,
                target = lambda m, s=self : voronoi_astar(
                    s.robot.strategy, s.match, aim_projection_ball
                )[1],
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

    def get_name(self):
        return f"<{self.robot.get_name()} Astar Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.05
        res = self.astar.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class DefendPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(
            match, 
            robot
        )

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {'max_speed': 2.5, 'max_angular': 3600, 'kp': 180}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|DefendBehaviour".format(self.__class__)
        )

        self.defend.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m, r=self.robot: (
                    m.ball.x + (math.cos(math.pi/3) if (m.ball.y < r.y) else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if (m.ball.y < r.y) else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.04,
                radius_max = 2,
                clockwise = lambda m, r=self.robot: (m.ball.y < r.y),
                decay=lambda x: 1,
                multiplier = 1
            )
        )

    def get_name(self):
        return f"<{self.robot.get_name()} Defend Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.05
        res = self.defend.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class SecondAttacker(Strategy):
    def __init__(self, match, name="RightAttacker"):
        controller_kwargs = {'max_speed': 2.5, 'max_angular': 6000, 'kp': 180}
        super().__init__(match, name, controller=PID_control, controller_kwargs=controller_kwargs)

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot, True)

        astar = AstarPlanning(self.match, self.robot)
        astar.start()

        defend_potentialfield = DefendPlanning(self.match, self.robot)
        defend_potentialfield.start()

        self.playerbook.add_play(astar)
        self.playerbook.add_play(defend_potentialfield)

        # Transicao para caso esteja perto da bola ( < 10 cm)
        next_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.60)
        # Transicao para caso esteja longe da bola ( > 20 cm)
        far_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.80, True)

        # astar.add_transition(next_to_ball_transition, defend_potentialfield)
        # defend_potentialfield.add_transition(far_to_ball_transition, astar)

        self.playerbook.set_play(defend_potentialfield)
    

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        return res
