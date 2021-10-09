from entities.coach.coach import BaseCoach
from entities import plays
import json


class Coach(BaseCoach):
    NAME = "LARC_2021"
    def __init__(self, match):
        super().__init__(match)

        self.positions = json.loads(open('foul_placements.json', 'r').read())

        self.playbook = plays.Playbook(self)

        main_play = plays.larc2021.MainPlay(self)
        penalty_play = plays.larc2021.PenaltyPlay(self)
        goalkick_play = plays.larc2021.GoalKickPlay(self)

        penalty_trigger = plays.OnPenaltyKick(self.match.game.referee, self.match.team_color)
        seven_seconds_trigger = plays.WaitForTrigger(7)

        goalkick_trigger = plays.OnGoalKick(self.match.game.referee, self.match.team_color)
        
        self.playbook.add_play(main_play)
        self.playbook.add_play(penalty_play)
        self.playbook.add_play(goalkick_play)

        main_play.add_transition(penalty_trigger, penalty_play)
        penalty_play.add_transition(seven_seconds_trigger, main_play)

        main_play.add_transition(goalkick_trigger, goalkick_play)
        goalkick_play.add_transition(seven_seconds_trigger, main_play)

        self.playbook.set_play(main_play)

    def _get_positions(self, foul, team_color, foul_color, quadrant):
        quad = quadrant
        foul_type = foul
        team = self.positions.get(team_color)
        foul = team.get(foul)

        if foul_type != "FREE_BALL":
            replacements = foul.get(foul_color, foul.get("POSITIONS"))
        else:
            replacements = foul.get(f"{quad}")
        return replacements

    
    def get_positions(self, foul, team_color, foul_color, quadrant):
        play_positioning = self.playbook.get_actual_play().get_positions(foul, team_color, foul_color, quadrant)
        if play_positioning:
            return play_positioning
        
        return self._get_positions(foul, team_color, foul_color, quadrant)
 

    def decide (self):
        self.playbook.update()
        # for r in self.match.robots:
        #     print(r.get_name(), r.strategy.name)


    