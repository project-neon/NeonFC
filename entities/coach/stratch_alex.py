import algorithms
import strategy

class Coach(object):
    def __init__(self, match):
        self.match = match

        self.deep_strategy = [
            strategy.alex.DeepPlay(self.match) for _ in self.match.robots
        ]
    
    def decide (self):
        robots = [r.robot_id for r in self.match.robots]

        for robot_id in robots:
            self.match.robots[robot_id].strategy = self.deep_strategy[robot_id]
            self.match.robots[robot_id].start()