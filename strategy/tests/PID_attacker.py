from strategy.BaseStrategy import Strategy
from controller import PID_control


class Attacker(Strategy):
    def __init__(self, match, name = 'PID-Attacker'):
        super().__init__(match,
            name=name,
            controller=PID_control
        )

    def start(self, robot=None):
        super().start(robot=robot)


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)


    def decide(self):
        return self.match.ball.x, self.match.ball.y
