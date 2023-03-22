from strategy.BaseStrategy import Strategy
from controller import NoController


class Move(Strategy):
    def __init__(self, match, v, w):
        super().__init__(match, 'Idle', NoController)
        self.v_max, self.w_max = v, w
        self.v, self.w = self.v_max, self.w_max

    def start(self, robot=None):
        super().start(robot=robot)

    def stop(self):
        self.v, self.w = 0, 0

    def move(self):
        self.v, self.w = self.v_max, self.w_max

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot

    def decide(self):
        print(self.v)
        return self.v, self.w
