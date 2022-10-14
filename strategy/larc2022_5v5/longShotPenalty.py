import math
import random
import algorithms
from controller.simple_LQR import TwoSidesLQR
from strategy.BaseStrategy import Strategy

class LongShotPenaltyTaker(Strategy):
    def __init__(self, match, name="long_shot", ctr_kwargs={'l': 0.0975}):
        super().__init__(
            match, 
            name, 
            controller=TwoSidesLQR,
            controller_kwargs=ctr_kwargs)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def start(self, robot=None):
        super().start(robot=robot)

        self.shootout = algorithms.fields.PotentialField(
            self.match,
            name="{}|ShootoutBehaviour".format(self.__class__)
        )

        self.shootout.add_field(
            algorithms.fields.PointField(
                self.match,
                target= lambda m: (m.ball.x, m.ball.y),
                radius=0.15,
                decay = lambda x: 1,
                multiplier = 10
            )
        )

    def get_position(self):
        angle_of_interest = 25
        dist_to_ball = 0.225
        shoot_side = random.choice([-1, 1])
        field_size = self.match.game.field.get_dimensions()

        if self.match.team_color == "blue":
            return {
                    "robot_id": 4, 
                    "x": field_size[0]/2 - 0.375 - math.cos(math.radians(angle_of_interest)) * dist_to_ball,
                    "y": shoot_side * math.sin(math.radians(angle_of_interest)) * dist_to_ball,
                    "orientation": - shoot_side * angle_of_interest
                }
        else:
            return {
                    "robot_id": 4, 
                    "x": - field_size[0]/2 + 0.375 + math.cos(math.radians(angle_of_interest)) * dist_to_ball,
                    "y": shoot_side * math.sin(math.radians(angle_of_interest)) * dist_to_ball,
                    "orientation": + shoot_side * angle_of_interest
                }


    def decide(self):
        behaviour = self.shootout
        return behaviour.compute([self.robot.x, self.robot.y])

