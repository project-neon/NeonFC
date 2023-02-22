from entities.coach.coach import BaseCoach
from entities import plays
import json

class Coach(BaseCoach):
    NAME = "IRON_2023_3V3"
    def __init__(self, match, coach_parameters={}):
        super().__init__(match)


        self.coach_parameters = coach_parameters
        self.positions = json.loads(open('foul_placements.json', 'r').read())
        self.playbook = plays.Playbook(self)

        main_play = plays.iron2023_3v3.MainPlay(self)
        penalty_play = plays.iron2023_3v3.PenaltyPlay(self, self.coach_parameters['penalty_taker']) 
        defend_penalty_play = plays.iron2023_3v3.DefendPenaltyPlay(self) 
        goalkick_play = plays.iron2023_3v3.GoalKickPlay(self) 
        freeball_play = plays.larc2021.FreeballPlay(self) 

        def_freeball_play = plays.larc2021.DefFreeballPlay(self) 

        penalty_trigger = plays.OnPenaltyKick(self.match.game.referee, self.match.team_color)
        defend_penalty_trigger = plays.OnPenaltyKick(self.match.game.referee, self.match.opposite_team_color)
        goalkick_trigger = plays.OnGoalKick(self.match.game.referee, self.match.team_color)
        freeball_trigger = plays.OnFreeBall(self.match.game.referee, self.match.team_color)

        # Contra bola parada da Bulls
        # deffreeball_trigger = plays.OnFreeBallDef(self.match.game.referee, self.match.team_color)

        penalty_seconds_trigger = plays.WaitForTrigger(12)
        defendpenalty_seconds_trigger = plays.WaitForTrigger(7)
        goalkick_seconds_trigger = plays.WaitForTrigger(11)
        freeball_seconds_trigger = plays.WaitForTrigger(9)

        self.playbook.add_play(main_play)
        self.playbook.add_play(penalty_play)
        self.playbook.add_play(defend_penalty_play)
        self.playbook.add_play(goalkick_play)
        self.playbook.add_play(freeball_play)
        self.playbook.add_play(def_freeball_play)

        main_play.add_transition(penalty_trigger, penalty_play)
        penalty_play.add_transition(penalty_seconds_trigger, main_play)

        main_play.add_transition(defend_penalty_trigger, defend_penalty_play)
        defend_penalty_play.add_transition(defendpenalty_seconds_trigger, main_play)

        main_play.add_transition(goalkick_trigger, goalkick_play)
        goalkick_play.add_transition(goalkick_seconds_trigger, main_play)

        main_play.add_transition(freeball_trigger, freeball_play)
        freeball_play.add_transition(freeball_seconds_trigger, main_play)

        # Contra bola parada da Bulls
        # main_play.add_transition(deffreeball_trigger, def_freeball_play)
        # def_freeball_play.add_transition(freeball_seconds_trigger, main_play)

        self.playbook.set_play(main_play)

    def _get_positions(self, foul, team_color, foul_color, quadrant):
        quad = quadrant
        foul_type = foul
        team = self.positions.get(team_color)
        foul = team.get(foul)
        if not foul:
            return None

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
        # print(self.playbook.actual_play)
        self.playbook.update()