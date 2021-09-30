import math
import algorithms
from commons import math as nfc_math
from strategy.BaseStrategy import Strategy
from controller import TwoSidesLQR
from strategy.DebugTools import DebugPotentialFieldStrategy


# class Attacker(DebugPotentialFieldStrategy):
class Attacker(Strategy):
    def __init__(self, match):
        self.ctrl_params = {"l": 0.08}
        super().__init__(match,
            name="UVF_Attacker",
            controller=TwoSidesLQR,
            controller_kwargs=self.ctrl_params
        )

    def start(self, robot=None):
        super().start(robot=robot)

        uvf_radius = 0.08 # 8 cm
        uvf_radius_2 = 0.08 # 8 cm

        tangential_speed = .8 # 80 cm/s

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

        def shifted_target_left(m, radius_2=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2 if abs(m.ball.y - field_w/2) > 0.12 else m.ball.y
            aim_point_x = field_h

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
            aim_point_y = field_w/2 if abs(m.ball.y - field_w/2) > 0.12 else m.ball.y
            aim_point_x = field_h

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
            aim_point_y = field_w/2 if abs(m.ball.y - field_w/2) > 0.12 else m.ball.y
            aim_point_x = field_h
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
            return speed * (dist + 0.5)

        def uvf_mean_contributtion_right(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2 if abs(m.ball.y - field_w/2) > 0.12 else m.ball.y
            aim_point_x = field_h
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
            return speed * (dist + 0.5)

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

        if nfc_math.point_in_rect(ball, goal_area):
            behaviour = self.wait
        else:
            behaviour = self.seek

        return behaviour.compute([self.robot.x, self.robot.y])

