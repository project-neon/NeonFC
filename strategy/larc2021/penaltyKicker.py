import algorithms
from controller.simple_LQR import TwoSidesLQR
from strategy.BaseStrategy import Strategy

class Shooter(Strategy):
    def __init__(self, match, name="shooter", ctr_kwargs={'l': 0.0975}):
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


    def decide(self):
        behaviour = self.shootout
        return behaviour.compute([self.robot.x, self.robot.y])

