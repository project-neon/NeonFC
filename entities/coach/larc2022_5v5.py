from entities.coach.coach import BaseCoach
from entities import plays
import json

class Coach(BaseCoach):
    NAME = "LARC_2022_5V5"
    def __init__(self, match):
        super().__init__(match)

        self.positions = json.loads(open('foul_placements5v5.json', 'r').read())
        self.playbook = plays.Playbook(self)

        main_play = plays.larc2022_5v5.MainPlay(self)
        penalty_play = plays.cbfrs2022_5v5.PenaltyPlay(self) # Add later this play to larc2022 5v5 package in plays
        defend_penalty_play = plays.cbfrs2022_5v5.DefendPenaltyPlay(self) # Add later this play to larc2022 5v5 package in plays
        goalkick_play = plays.cbfrs2022_5v5.GoalKickPlay(self) # Add later this play to larc2022 5v5 package in plays

        penalty_trigger = plays.OnPenaltyKick(self.match.game.referee, self.match.team_color)
        defend_penalty_trigger = plays.OnPenaltyKick(self.match.game.referee, self.match.opposite_team_color)
        goalkick_trigger = plays.OnGoalKick(self.match.game.referee, self.match.team_color)

        penalty_seconds_trigger = plays.WaitForTrigger(10)
        defendpenalty_seconds_trigger = plays.WaitForTrigger(9)
        goalkick_seconds_trigger = plays.WaitForTrigger(13)

        self.playbook.add_play(main_play)
        self.playbook.add_play(penalty_play)
        self.playbook.add_play(defend_penalty_play)
        self.playbook.add_play(goalkick_play)

        main_play.add_transition(penalty_trigger, penalty_play)
        penalty_play.add_transition(penalty_seconds_trigger, main_play)

        main_play.add_transition(defend_penalty_trigger, defend_penalty_play)
        defend_penalty_play.add_transition(defendpenalty_seconds_trigger, main_play)

        main_play.add_transition(goalkick_trigger, goalkick_play)
        goalkick_play.add_transition(goalkick_seconds_trigger, main_play)

        self.playbook.set_play(main_play)