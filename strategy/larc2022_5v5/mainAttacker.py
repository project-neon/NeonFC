import math
import threading

from algorithms.potential_fields import fields
from controller.simple_LQR import TwoSidesLQR
from entities import plays

from strategy.BaseStrategy import Strategy
from controller import PID_control
from commons import math as nfc_math

from strategy.larc2022_5v5.commons import AstarPlanning, LimitCyclePlanning, aim_projection_ball
from strategy.utils.player_playbook import OnCorners, OnInsideBox, OnNextTo, OnStuckTrigger, PlayerPlay, PlayerPlaybook

def aim_behind_ball(strategy):
    b = strategy.match.ball
    ball = [b.x, b.y - 0.05]
    return ball

class CarryPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Carry Potential Planning>"

    def start_up(self):
            super().start_up()
            controller = TwoSidesLQR
            controller_kwargs = {'l': 0.185}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        self.carry = fields.PotentialField(
            self.match,
            name="{}|CarryBehaviour".format(self.__class__)
        )
        height, width = self.match.game.field.get_dimensions()

        self.carry.add_field(
            fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.05, # 30cm
                decay = lambda x: 1,
                multiplier = lambda m: max(0.80, math.sqrt(m.ball.vx**2 + m.ball.vy**2) + 0.25) # 50 cm/s
            )
        )
        self.carry.add_field(
            fields.LineField(
                self.match,
                target= lambda m: (
                    m.ball.x - math.cos(math.atan2((width/2-m.ball.y), (height - m.ball.x))) * 0.025, 
                    m.ball.y - math.sin(math.atan2((width/2-m.ball.y), (height - m.ball.x))) * 0.025
                ),
                theta=lambda m: ( -math.atan2((width/2-m.ball.y), (height - m.ball.x))),
                line_size = 1,
                line_size_single_side = True,
                line_dist = 0.15,
                line_dist_max = 0.15,
                decay = lambda x: x**2,
                multiplier = 1.3 # 75 cm/s
            )
        )

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        res = self.carry.compute(robot_pos)
        return res

class WingPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Wing Potential Planning>"

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2, 'max_angular': 4800}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        self.seek = fields.PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        uvf_radius = 0.075 # 7.5 cm
        uvf_radius_2 = 0.075 # 7.5 cm
        
        _, field_w = self.match.game.field.get_dimensions()

        def shifted_target_right(m, radius=uvf_radius_2):
            which_side = 1 if (m.ball.y < field_w/2) else -1
            ball = [m.ball.x, m.ball.y + which_side * 0.05]
            
            ball_x = ball[0]
            ball_y = ball[1]

            pos_x = (
                ball_x
            )

            pos_y = (
                ball_y + which_side * uvf_radius_2
            )

            return [pos_x, pos_y]
        
        self.seek.add_field(
            fields.TangentialField(
                self.match,
                target=shifted_target_right,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = lambda m: (m.ball.y < field_w/2),
                decay=lambda x: 1,
                multiplier = 1,
                K = 1/250
            )
        )

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.3
        res = self.seek.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class AttackingWaitPlanning(PlayerPlay):
    def __init__(self, match, robot):
        threading.Thread.__init__(self)
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
        controller_kwargs = {'max_speed': 2, 'max_angular': 4800}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        self.avoid = fields.PotentialField(
            self.match,
            name="{}|AstarBehaviour".format(self.__class__)
        )
        height, width = self.match.game.field.get_dimensions()

        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue
            self.avoid.add_field(
                fields.PointField(
                    self.match,
                    target = lambda m, r=robot: (
                        r.x,
                        r.y
                    ),
                    radius = .2,
                    radius_max = .2,
                    decay = lambda x: -1,
                    multiplier = 1
                )
            )
        self.avoid.add_field(
            fields.PointField(
                self.match,
                target = [height/2, width/2],
                radius = .3,
                decay = lambda x: 1,
                multiplier = 1.5
            )
        )

    def get_name(self):
        return f"<{self.robot.get_name()} Avoid Area Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.3
        res = self.avoid.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class AvoidRobotsPlanning(PlayerPlay):
    def __init__(self, match, robot):
        threading.Thread.__init__(self)
        super().__init__(
            match, 
            robot
        )

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {'max_speed': 3.5, 'max_angular': 4800}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        self.avoid = fields.PotentialField(
            self.match,
            name="{}|AvoidBehaviour".format(self.__class__)
        )

        field_h, field_w = self.match.game.field.get_dimensions()

        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue
            self.avoid.add_field(
                fields.PointField(
                    self.match,
                    target = lambda m, r=robot: (
                        r.x,
                        r.y
                    ),
                    radius = .2,
                    radius_max = .2,
                    decay = lambda x: -1,
                    multiplier = 1
                )
            )
        self.avoid.add_field(
            fields.PointField(
                self.match,
                target = [field_h/2, field_w/2],
                radius = .1,
                decay = lambda x: 1,
                multiplier = 1.2
            )
        )
        

    def get_name(self):
        return f"<{self.robot.get_name()} Avoid Robots Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.3
        res = self.avoid.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class PushPotentialFieldPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Push Potential Planning>"

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2, 'max_angular': 4800, 'kd': 1, 'kp': 200}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        self.seek = fields.PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        uvf_radius = 0.075 # 7.5 cm
        uvf_radius_2 = 0.075 # 7.5 cm

        tangential_speed = lambda m: max(.6, (m.ball.vx**2 + m.ball.vy**2)**.5 + .3 ) # 8 cm/s


        def shifted_target_left(m, radius_2=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2
            aim_point_x = field_h

            ball = [m.ball.x, m.ball.y]
            goal_pos = [
                m.game.field.get_dimensions()[0] + 0.04,
                m.game.field.get_dimensions()[1]/2
            ]

            dir_to_goal_vector = [
                goal_pos[0] - ball[0], 
                goal_pos[1] - ball[1]
            ]
            angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

            ball_x = ball[0] - 0.08 * math.cos(angle)
            ball_y = ball[1] - 0.08 * math.sin(angle)

            pos_x = (
                ball_x -
                math.cos(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2)*radius_2
            )

            pos_y = (
                ball_y -
                math.sin(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2)*radius_2
            )

            return [pos_x, pos_y]

        def shifted_target_right(m, radius=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2
            aim_point_x = field_h

            ball = [m.ball.x, m.ball.y]
            goal_pos = [
                m.game.field.get_dimensions()[0] + 0.04,
                m.game.field.get_dimensions()[1]/2
            ]

            dir_to_goal_vector = [
                goal_pos[0] - ball[0], 
                goal_pos[1] - ball[1]
            ]
            angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

            ball_x = ball[0] - 0.08 * math.cos(angle)
            ball_y = ball[1] - 0.08 * math.sin(angle)

            pos_x = (
                ball_x +
                math.cos(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2) * radius
            )

            pos_y = (
                ball_y +
                math.sin(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2) * radius
            )

            return [pos_x, pos_y]
        
        def uvf_mean_contributtion_left(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2

            aim_point_x = field_h  + 0.04
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
            aim_point_y = field_w/2

            aim_point_x = field_h  + 0.04
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

        # field_h, field_w = self.match.game.field.get_dimensions()
        self.seek.add_field(
            fields.TangentialField(
                self.match,
                target=shifted_target_left,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_left,
                K = 1/250
            )
        )

        self.seek.add_field(
            fields.TangentialField(
                self.match,
                target=shifted_target_right,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_right,
                K = 1/250
            )
        )

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.3
        res = self.seek.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.planning = None
        self.path = []

        self.actual_iteration = -1

        self.is_on_attack = False

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot, True)

        # Criando Path Planning baseado em Astar
        astar = AstarPlanning(self.match, self.robot)
        astar.start()

        # Criando Potential Field
        push_potentialfield = PushPotentialFieldPlanning(self.match, self.robot)
        push_potentialfield.start()

        avoid_potentialfield = AvoidRobotsPlanning(self.match, self.robot)
        avoid_potentialfield.start()

        wait_potentialfield = AttackingWaitPlanning(self.match, self.robot)
        wait_potentialfield.start()

        wing_potentialfield = WingPlanning(self.match, self.robot)
        wing_potentialfield.start()

        # carry_potentialfield = CarryPlanning(self.match, self.robot)
        # carry_potentialfield.start()


        # Adiciona plays no livro do jogador
        self.playerbook.add_play(astar)
        self.playerbook.add_play(push_potentialfield)
        self.playerbook.add_play(avoid_potentialfield)
        self.playerbook.add_play(wait_potentialfield)
        self.playerbook.add_play(wing_potentialfield)
        # self.playerbook.add_play(carry_potentialfield)

        # Transicao para caso esteja perto da bola ( < 40 cm)
        next_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.40)
        # Transicao para caso esteja longe da bola ( > 60 cm)
        far_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.60, True)
        inside_defender_area_transition = OnInsideBox(self.match, [0, 0.35, 0.15, 0.4])
        outside_defender_area_transition = OnInsideBox(self.match, [0, 0.35, 0.15, 0.4], True)
        stuck_transition = OnStuckTrigger(self.robot, 1/2)
        wait_transition = plays.WaitForTrigger(2/3)
        corners_transition = OnCorners(self.match, [0.1, 1.65])
        out_corners_transition = OnCorners(self.match, [0.1, 1.65], True)
        # goal_zone_transition = OnInsideBox(self.match, [1.85, 0.5, 0.4, 1.3 - 0.5])
        # out_goal_zone_transition = OnInsideBox(self.match, [1.85, 0.5, 0.4, 1.3 - 0.5], True)

        # Adiciona caminhos de ida e volta com transicoes
        # astar.add_transition(next_to_ball_transition, push_potentialfield)
        # astar.add_transition(stuck_transition, avoid_potentialfield)
        # astar.add_transition(inside_defender_area_transition, wait_potentialfield)
        # astar.add_transition(goal_zone_transition, carry_potentialfield)

        # push_potentialfield.add_transition(far_to_ball_transition, astar)
        # push_potentialfield.add_transition(stuck_transition, avoid_potentialfield)
        # push_potentialfield.add_transition(corners_transition, wing_potentialfield)
        # push_potentialfield.add_transition(inside_defender_area_transition, wait_potentialfield)
        # push_potentialfield.add_transition(goal_zone_transition, carry_potentialfield)

        # avoid_potentialfield.add_transition(wait_transition, astar)
        # wait_potentialfield.add_transition(outside_defender_area_transition, astar)
        # wing_potentialfield.add_transition(out_corners_transition, push_potentialfield)

        # carry_potentialfield.add_transition(out_goal_zone_transition, astar)
        

        # Estado inicial é o astar
        self.playerbook.set_play(push_potentialfield)

    def decide(self):
        res = self.playerbook.update()
        return res
