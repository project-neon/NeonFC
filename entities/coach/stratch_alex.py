import algorithms
import strategy

class Coach(object):
    def __init__(self, match):
        self.match = match

        self.test_strategy = [
            strategy.alex.DefensivePlay(self.match) for _ in self.match.robots
        ]

        self.goalkeeper = [
            strategy.larc2020.GoalKeeper(self.match) for _ in self.match.robots
        ]
    
    def decide (self):
        robots = [r.robot_id for r in self.match.robots]

        for robot_id in robots:
            if self.match.robots[robot_id].strategy is not None:
                continue

            if robot_id in [0, 1]:
                self.match.robots[robot_id].strategy = self.test_strategy[robot_id]
                self.match.robots[robot_id].start()
            else:
                self.match.robots[robot_id].strategy = self.goalkeeper[robot_id]
                self.match.robots[robot_id].start()