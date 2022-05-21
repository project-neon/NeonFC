import math
import random

import strategy
from entities.plays.rcx2022.mainPlay import MainPlay


class GoalKickPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)
        self.constraints = [
            (strategy.larc2021.Shooter(self.match, "ShooterGoalkeeper"), self._elect_goalkeeper),
            (strategy.tests.SpinnerAttacker(self.match), self._elect_attacker),
            (strategy.tests.MidFielderSupporter(self.match, attacker="SpinnerAttacker"), self._elect_midfielder)
        ]

    def freeze_positions(self, constraints, robots):
        return constraints, robots

    def get_positions(self, foul, team_color, foul_color, quadrant):
        angle_of_interest = 115
        dist_to_ball = 0.225

        replacements = self.coach._get_positions(foul, team_color, foul_color, quadrant)

        if foul == "GOAL_KICK" and foul_color == team_color:
            kicker_pos = list(filter(lambda r: r["robot_id"] == 0, replacements))
            if len(kicker_pos):
                replacements.remove(kicker_pos[0])
            field_size = self.match.game.field.get_dimensions()
            if team_color  == "BLUE":
                replacements.append(
                    {
                        "robot_id": 0, 
                        "x": -field_size[0]/2 + 0.1,
                        "y": 0.2,
                        "orientation": 60
                    }
                )
            else:
                replacements.append(
                    {
                        "robot_id": 0, 
                        "x": field_size[0]/2 - 0.1,
                        "y": 0.2,
                        "orientation": angle_of_interest
                    }
                )
                
            return replacements

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()
