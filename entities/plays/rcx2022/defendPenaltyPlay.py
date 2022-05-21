import math
import random

import strategy
from entities.plays.rcx2022.mainPlay import MainPlay


class DefendPenaltyPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)
        self.constraints = [
            (strategy.larc2021.Shooter(self.match, "ShooterGoalkeeper"), self._elect_goalkeeper),
            (strategy.tests.SpinnerAttacker(self.match), self._elect_attacker),
            (strategy.tests.MidFielderSupporter(self.match, attacker="SpinnerAttacker"), self._elect_midfielder)
        ]

    def freeze_positions(self, constraints, robots):
        return constraints, robots

    def _can_play(self):
        return self.match.game.referee.can_play()

    def get_positions(self, foul, team_color, foul_color, quadrant):
        replacements = self.coach._get_positions(foul, team_color, foul_color, quadrant)

        if foul == "PENALTY_KICK" and foul_color != team_color:
            kicker_pos = list(filter(lambda r: r["robot_id"] == 0, replacements))
            if len(kicker_pos):
                replacements.remove(kicker_pos[0])
            if team_color  == "BLUE":
                replacements.append(
                    {
                        "robot_id": 0, 
                        "x": -0.71,
                        "y": 0,
                        "orientation": 0
                    }
                )
            else:
                replacements.append(
                    {
                        "robot_id": 0, 
                        "x": 0.71,
                        "y": 0,
                        "orientation": 0
                    }
                )
                
            return replacements

    def update(self):
        super().update()
