from entities.coach.coach import BaseCoach
from entities import plays

class Coach(BaseCoach):
    NAME = "IRON_2021"
    def __init__(self, match):
        super().__init__(match)

        self.playbook = plays.Playbook(self)

        one_to_one_play = plays.OneOneOnePlay(self)
        unstuck_play = plays.UnstuckPlay(self)

        stuck_trigger = plays.StuckRobotsTrigger(stuck_strategies=['attacker', 'midfielder'])
        wait_for_trigger = plays.WaitForTrigger(5)

        one_to_one_play.add_transition(stuck_trigger, unstuck_play)
        unstuck_play.add_transition(wait_for_trigger, one_to_one_play)

        self.playbook.add_play(one_to_one_play)
        self.playbook.add_play(unstuck_play)

        self.playbook.set_play(one_to_one_play)
    
    def decide (self):
        self.playbook.update()

    