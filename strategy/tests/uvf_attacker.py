import math
import random
import algorithms
from commons import math as nfc_math
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

# class Attacker(DebugPotentialFieldStrategy):
class Attacker(Strategy):
    def __init__(self, match):
        super().__init__(match,
        name="UVF_Attacker"
        )



    def start(self, robot=None):
        super().start(robot=robot)

        uvf_radius = 0.05 # 5 cm
        uvf_radius_2 = 0.1 # 10 cm

        tangential_speed = 0.5 # 50 cm/s

        """
        MTG-UVF: move to goal univector field

        2 campos tangenciais
        target: deslocamento para o lado da bola em relacao ao gol
        multiplier: media ponderada em relacao ao raio
        """

        self.field = algorithms.fields.PotentialField(
            self.match,
            name="{}|FieldBehaviour".format(self.__class__)
        )

        def shifted_target_left(m, radius_2=uvf_radius_2):
            field_w, field_h = m.game.field.get_dimensions()

            pos_x = (
                m.ball.x -
                math.cos(math.atan2((field_w/2-m.ball.y), (field_h - m.ball.x))+ math.pi/2)*radius_2
            )

            pos_y = (
                m.ball.y -
                math.sin(math.atan2((field_w/2-m.ball.y), (field_h - m.ball.x))+ math.pi/2)*radius_2
            )

            return [pos_x, pos_y]

        def shifted_target_right(m, radius=uvf_radius_2):
            field_w, field_h = m.game.field.get_dimensions()

            pos_x = (
                m.ball.x +
                math.cos(math.atan2((field_w/2-m.ball.y), (field_h - m.ball.x))+ math.pi/2) * radius
            )

            pos_y = (
                m.ball.y +
                math.sin(math.atan2((field_w/2-m.ball.y), (field_h - m.ball.x))+ math.pi/2) * radius
            )

            return [pos_x, pos_y]

        
        def uvf_mean_contributtion_left(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_w, field_h = m.game.field.get_dimensions()
            target = [
                m.ball.x,
                m.ball.y
            ]

            dist_target_robot = ( (pos[0] - target[0])**2 + (pos[1] - target[1])**2)**.5

            target2 = [
                m.ball.x +
                math.cos(math.atan2((field_w/2-m.ball.y), (field_h - m.ball.x))) * 0.025,
                m.ball.y +
                math.sin(math.atan2((field_w/2-m.ball.y), (field_h- m.ball.x))) * 0.025
            ]

            dist = nfc_math.distance_to_line(pos[0], pos[1], target[0], target[1], target2[0], target2[1])
            if radius < dist_target_robot:
                dist = max(0, min(dist, radius))/ (radius * 2)
                return speed * (dist + 0.5)
            dist = max(0, min(dist, radius))/ (radius)
            return speed * dist

        def uvf_mean_contributtion_right(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_w, field_h = m.game.field.get_dimensions()
            target = [
                m.ball.x,
                m.ball.y
            ]

            dist_target_robot = ( (pos[0] - target[0])**2 + (pos[1] - target[1])**2)**.5

            target2 = [
                m.ball.x +
                math.cos(math.atan2((field_w/2-m.ball.y), (field_h - m.ball.x))) * 0.025,
                m.ball.y +
                math.sin(math.atan2((field_w/2-m.ball.y), (field_h- m.ball.x))) * 0.025
            ]

            dist = -nfc_math.distance_to_line(pos[0], pos[1], target[0], target[1], target2[0], target2[1])
            if radius < dist_target_robot:
                dist = max(0, min(dist, radius))/ (radius * 2)
                return speed * (dist + 0.5)
            dist = max(0, min(dist, radius))/ (radius)
            return speed * dist
                


        self.field.add_field(
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

        self.field.add_field(
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


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):
        behaviour = self.field

        # return super().decide(behaviour)
        return behaviour.compute([self.robot.x, self.robot.y])

