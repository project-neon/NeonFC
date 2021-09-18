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
        
        self.playbook.add_play(main_play)
        self.playbook.set_play(main_play)
    
    # def get_positions(self, foul, team_color, foul_color):
    #     team = self.positions.get(team_color)
    #     foul = team.get(foul)
    #     replacements = foul.get(foul_color, foul.get("POSITIONS"))
    #     return replacements

    def decide (self):
        self.playbook.update()

    