from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

# class MainGoalkeeper(DebugPotentialFieldStrategy):
class Goalkeeper(Strategy):
    def __init__(self, match, name="MainGoalkeeper"):
        super().__init__(match, name)

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):

        return 0, 0
        # return super().decide(behaviour)