import math
import random

import strategy
from entities.plays.larc2021.mainPlay import MainPlay


class FreeballPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)
        self.constraints = [
            (strategy.larc2022_5v5.GoalKeeper(self.match, 'Goalkeeper'), self._elect_goalkeeper),
            (strategy.larc2021.Shooter(self.match, "Shooter"), self._elect_attacker),
            (strategy.larc2022_5v5.MainAttacker(self.match, 'MainAttacker'), self._elect_midfielder)
        ]

    def freeze_positions(self, constraints, robots):
        return constraints, robots


    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()
