import math
import os
import random

from entities.plays.iron2022_3v3.mainPlay import MainPlay
import strategy

from strategy.larc2022_5v5.shortShotPenaltyTaker import ShortShotPenaltyTaker
from strategy.larc2022_5v5.longShotPenalty import LongShotPenaltyTaker



class PenaltyPlay(MainPlay):
    def __init__(self, coach, penalty_taker):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach

        self.penalty_taker = os.environ.get('PENALTY_TAKER', penalty_taker) 
        self.PENALTY_TAKER_ID = 2

        if self.penalty_taker == 'long_shot':
            self.gk = LongShotPenaltyTaker(self.match)
        elif self.penalty_taker == 'short_shot':
            self.gk = ShortShotPenaltyTaker(self.match)

        self.strategies = [
            strategy.larc2022_5v5.GoalKeeper(self.match, 'Goalkeeper'),
            strategy.larc2022_5v5.RadialDefender(self.match, 'Defender1'),
            self.gk,
        ]
        

    def get_positions(self, foul, team_color, foul_color, quadrant):

        replacements = self.coach._get_positions(foul, team_color, foul_color, quadrant)

        if foul == "PENALTY_KICK" and foul_color == team_color:
            kicker_pos = list(filter(lambda r: r["robot_id"] == self.PENALTY_TAKER_ID, replacements))
            if len(kicker_pos):
                replacements.remove(kicker_pos[0])
            if team_color  == "BLUE":
                replacements.append(
                    self.gk.get_position()
                )
            else:
                replacements.append(
                    self.gk.get_position()
                )
                
            return replacements

        return None

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()
