from entities.coach.coach import BaseCoach
import strategy

class Coach(BaseCoach):
    NAME = "DEV_ALEX"
    def __init__(self, match):
        self.match = match

        self.attacker = [
            strategy.alex.DefensivePlay(self.match) for _ in self.match.robots
        ]

        self.mock = [
            strategy.tests.Idle(self.match) for _ in self.match.robots
        ]
    
    def decide (self):
        robots = [r.robot_id for r in self.match.robots]

        for robot_id in robots:
            if robot_id == 0:
                if self.match.robots[robot_id].strategy is None:
                    self.match.robots[robot_id].strategy = self.attacker[robot_id]
                    self.match.robots[robot_id].start()
            else:
                if self.match.robots[robot_id].strategy is None:
                    self.match.robots[robot_id].strategy = self.mock[robot_id]
                    self.match.robots[robot_id].start()