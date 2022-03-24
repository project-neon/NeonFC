from entities.coach.coach import BaseCoach

import strategy
import json

class Coach(BaseCoach):
    NAME = "TEST_NEWGK"
    def __init__(self, match):
        super().__init__(match)

        self.midfielder_strategy = strategy.tests.MidFielder2(self.match)
        self.goalkeeper_strategy = strategy.tests.newGoalKeeper(self.match)
        self.attacker_strategy = strategy.tests.UVFAttacker(self.match)

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

    def get_positions(self, foul, team_color, foul_color, quadrant):
        quad = quadrant
        foul_type = foul
        team = self.positions.get(team_color)
        foul = team.get(foul)
        if foul_type != "FREE_BALL":
            replacements = foul.get(foul_color, foul.get("POSITIONS"))
        else:
            replacements = foul.get(f"{quad}")
        return replacements