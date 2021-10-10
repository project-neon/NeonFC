import math
import random

import strategy
from entities.plays.larc2021.mainPlay import MainPlay


class GoalKickPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)
        self.constraints = [
            (strategy.larc2021.Shooter(self.match, "ShooterGoalkeeper"), self._elect_goalkeeper),
            (strategy.tests.UVFAttacker(self.match), self._elect_attacker),
            (strategy.tests.newMidFielder(self.match, ""), self._elect_midfielder),
            (strategy.tests.Defender(self.match, "Left"), self._elect_goalkeeper),
            (strategy.tests.Defender(self.match, "Right"), self._elect_goalkeeper),
        ]

    def freeze_positions(self, constraints, robots):
        return constraints, robots

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()
