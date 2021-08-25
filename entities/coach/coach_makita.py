from entities.coach.coach import BaseCoach

import strategy

class Coach(BaseCoach):
    NAME = "TEST_MAKITA"
    def __init__(self, match):
        super().__init__(match)

        self.attacker_strategy = strategy.tests.mktAttacker(self.match)
        self.midfielder_strategy = strategy.tests.Idle(self.match)
        self.goalkeeper_strategy = strategy.tests.mktGoalKeeper(self.match)

    def decide(self):
        robots = [r.robot_id for r in self.match.robots]
        strategies = {
            self.goalkeeper_strategy, self.attacker_strategy, self.midfielder_strategy
        }

        for robot, strategy in zip(robots, strategies):
            if self.match.robots[robot].strategy is not None:

                continue
        
            self.match.robots[robot].strategy = strategy
            self.match.robots[robot].start()
