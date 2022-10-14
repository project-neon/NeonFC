import random
import math
import strategy
from entities.plays.cbfrs2022_5v5.mainPlay import MainPlay

class PenaltyPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)

        self.constraints = [
            (strategy.cbfrs2022_5v5.GoalKeeper(self.match), self._elect_goalkeeper),
            (strategy.tests.UVFAttacker(self.match), self._elect_leftattacker),
            (strategy.cbfrs2022_5v5.LeftWing(self.match), self._elect_leftwing),
            (strategy.cbfrs2022_5v5.RightWing(self.match), self._elect_rightwing),
            (strategy.cbfrs2022_5v5.RightAttacker(self.match), self._elect_rightattacker)
        ]

    def freeze_positions(self, constraints, robots):
        return constraints, robots

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()