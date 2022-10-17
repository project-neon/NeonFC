import math
import random
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR
from commons import math as nfc_math
from algorithms.potential_fields import fields
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay

class PushPotentialFieldPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Push Potential Planning>"

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 3, 'max_angular': 4800}
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

            ball_x = ball[0] - 0.07 * math.cos(angle)
            ball_y = ball[1] - 0.07 * math.sin(angle)

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

            ball_x = ball[0] - 0.07 * math.cos(angle)
            ball_y = ball[1] - 0.07 * math.sin(angle)

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

class ShortShotPenaltyTaker(Strategy):
    def __init__(self, match, name="short_shot", ctr_kwargs={'l': 0.0975}, robot_id=2):
        super().__init__(
            match, 
            name, 
            controller=TwoSidesLQR,
            controller_kwargs=ctr_kwargs)

        self.robot_id = robot_id

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def start(self, robot=None):
        super().start(robot=robot)

        self.shootout = PushPotentialFieldPlanning(self.match, self.robot)
        self.shootout.start()
        self.shootout.start_up()


    def get_position(self):
        angle_of_interest = 26.5
        dist_to_ball = 0.07
        shoot_side = random.choice([-1, 1])
        field_size = self.match.game.field.get_dimensions()

        if self.match.team_color == "blue":
            return {
                    "robot_id": self.robot_id, 
                    "x": field_size[0]/2 - field_size[0]/6 - math.cos(math.radians(angle_of_interest)) * dist_to_ball,
                    "y": shoot_side * math.sin(math.radians(angle_of_interest)) * dist_to_ball,
                    "orientation": - shoot_side * angle_of_interest
                }
        else:
            return {
                    "robot_id": self.robot_id, 
                    "x": - field_size[0]/2 + field_size[0]/6 + math.cos(math.radians(angle_of_interest)) * dist_to_ball,
                    "y": shoot_side * math.sin(math.radians(angle_of_interest)) * dist_to_ball,
                    "orientation": + shoot_side * angle_of_interest
                }


    def decide(self):
        return self.shootout.update()

