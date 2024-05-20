from strategy.BaseStrategy import Strategy
from controller import NoController

class Foward(Strategy):
    def __init__(self, match):
        super().__init__(match, 'Idle', controller=NoController)


    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot


    def decide(self):
        return 30, 0
