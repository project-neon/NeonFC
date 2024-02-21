import math
from collections import deque
from strategy.BaseStrategy import Strategy
from controller import PID_control, SimpleLQR, TwoSidesLQR, UniController
from NeonPathPlanning.potential_fields import PotentialField, PointField, LineField, TangentialField

class Fields_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(
            match, 
            name="Potential_Fields_Test", 
            controller=TwoSidesLQR, 
            controller_kwargs={'l': 0.0135}
            )

        self.circuit = [(0.375, 0.25), (1.125, 0.25), (0.75, 0.65), (1.125, 1.05), (0.375, 1.05)]
        self.circuit = deque(self.circuit)
        self.dl = 0.000001

    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]

    def start(self, robot=None):
        super().start(robot=robot)

        self.test_field = PotentialField(
            self.match,
            name=f'{self.__class__}|TestFieldBehaviour'
        )

        # self.test_field.add_field(
        #     PointField(
        #         self.match,
        #         target = (.75, .65),
        #         radius = .3,
        #         decay = lambda x: x,
        #         multiplier = .8,
        #     )
        # )

        self.test_field.add_field(
            LineField()
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        behaviour = self.test_field

        return behaviour.compute([self.robot.x, self.robot.y])
