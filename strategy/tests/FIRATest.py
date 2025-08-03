import time

from controller.noController import NoController
from strategy.BaseStrategy import Strategy


class FiraTest(Strategy):
    def __init__(self, match):
        self.lt = time.time()
        super().__init__(match, "FiraTest", controller=NoController)

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        return 20, 20  # 0, 10
