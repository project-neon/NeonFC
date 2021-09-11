from entities.coach.coach import BaseCoach

import strategy
import json

class Coach(BaseCoach):
    NAME = "TEST_NEWGK"
    def __init__(self, match):
        super().__init__(match)

        self.attacker_strategy = strategy.iron2021.Attacker(self.match)
        self.midfielder_strategy = strategy.iron2021.MidFielder(self.match)
        self.goalkeeper_strategy = strategy.tests.newGoalKeeper(self.match)

        self.positions = json.loads(open('foul_placements.json', 'r').read())

    def decide(self):
        self.match.robots[0].strategy = self.goalkeeper_strategy
        self.match.robots[0].start()
        self.match.robots[1].strategy = self.attacker_strategy
        self.match.robots[1].start()
        self.match.robots[2].strategy = self.midfielder_strategy
        self.match.robots[2].start()
        '''
        robots = [r.robot_id for r in self.match.robots]
        strategies = {
            self.goalkeeper_strategy, self.attacker_strategy, self.midfielder_strategy
        }

        for robot, strategy in zip(robots, strategies):
            if self.match.robots[robot].strategy is not None:

                continue
        
            self.match.robots[robot].strategy = strategy
            self.match.robots[robot].start()'''

    def get_positions(self, foul, team_color):
        foul = self.positions.get(foul)
        replacements = foul.get(team_color, foul.get("POSITIONS"))
        return replacements
