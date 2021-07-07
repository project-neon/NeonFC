from entities.coach.coach import BaseCoach
import strategy

class Coach(BaseCoach):
    def __init__(self, match):
        super().__init__(match, "Experiment-astar")
        self.test_strategy = strategy.tests.AstarAttacker(self.match)

        self.idle_strategy = [
            strategy.tests.Idle(self.match) for _ in self.match.robots
        ]
    
    def decide (self):
        robots = [r.robot_id for r in self.match.robots]

        for robot_id in robots:
            if self.match.robots[robot_id].strategy is not None:
                continue

            if robot_id == 0:
                self.match.robots[robot_id].strategy = self.test_strategy
                self.match.robots[robot_id].start()
            else:
                self.match.robots[robot_id].strategy = self.idle_strategy[robot_id]
                self.match.robots[robot_id].start()