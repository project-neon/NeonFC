import algorithms
from strategy.BaseStrategy import Strategy

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, 'Idle')


    def start(self, robot=None):
        super().start(robot=robot)

        self.test_field = algorithms.fields.PotentialField(
            self.match,
            name="{}|AnyBehaviour".format(self.__class__)
        )

        self.test_field.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (0.750, 0.650), # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: x,
                multiplier = 0.5
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return self.test_field.compute([self.robot.x, self.robot.y])

