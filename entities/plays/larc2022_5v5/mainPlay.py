import math
import strategy
from entities.plays.playbook import Play

class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self.strategies = [
            strategy.cbfr2022.MainGoalkeeper(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match)
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()