import strategy
from entities.plays.cbfrs2022_5v5.mainPlay import MainPlay

class GoalKickPlay(MainPlay):
    def __init__(self, coach):
        super().__init__(coach)

        self.constraints = [
            (strategy.larc2021.Shooter(self.match), self._elect_goalkeeper),
            (strategy.larc2021.Shooter(self.match), self._elect_leftattacker),
            (strategy.cbfrs2022_5v5.LeftWing(self.match), self._elect_leftwing),
            (strategy.cbfrs2022_5v5.RightWing(self.match), self._elect_rightwing),
            (strategy.cbfrs2022_5v5.RightAttacker(self.match), self._elect_rightattacker)
        ]

    def get_positions(self, foul, team_color, foul_color, quadrant):
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
                        "y": -0.14,
                        "orientation": 67 + 45
                    }
                )
            else:
                replacements.append(
                    {
                        "robot_id": 0, 
                        "x": field_size[0]/2 - 0.1,
                        "y": -0.14,
                        "orientation": 67
                    }
                )
                
            return replacements
        
        return None

    def freeze_positions(self, constraints, robots):
        return constraints, robots

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()