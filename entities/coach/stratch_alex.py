import json
import math

import strategy
from entities.coach.coach import BaseCoach


# class Coach(BaseCoach):
#     NAME = "DEV_ALEX"
#     def __init__(self, match):
#         self.match = match

#         self.attacker = [
#             strategy.alex.OffensivePlay(self.match) for _ in self.match.robots
#         ]

#         self.mock = [
#             strategy.tests.Idle(self.match) for _ in self.match.robots
#         ]
    
#     def decide (self):
#         robots = [r.robot_id for r in self.match.robots]

#         for robot_id in robots:
#             if robot_id == 0:
#                 if self.match.robots[robot_id].strategy is None:
#                     self.match.robots[robot_id].strategy = self.attacker[robot_id]
#                     self.match.robots[robot_id].start()
#             else:
#                 if self.match.robots[robot_id].strategy is None:
#                     self.match.robots[robot_id].strategy = self.mock[robot_id]
#                     self.match.robots[robot_id].start()

class Coach(BaseCoach):
    NAME = "DEV_ALEX"
    def __init__(self, match):
        super().__init__(match)

        self.constraints = [
            #estratégia - função eleitora - prioridade
            (strategy.larc2020.GoalKeeper(self.match), self.elect_goalkeeper, 0),
            (strategy.alex.OffensivePlay(self.match), self.elect_attacker, 0),
            (strategy.larc2020.MidFielder(self.match), self.elect_midfielder, 0)
        ]
    
    def decide (self):
        robots = [r.robot_id for r in self.match.robots]
        for strategy, fit_fuction, _ in self.constraints:
            elected = -1
            best_fit = -99999
            for robot_id in robots:
                robot_fit = fit_fuction(self.match.robots[robot_id])
                if (robot_fit > best_fit):
                    best_fit = robot_fit
                    elected = robot_id
            if self.match.robots[elected].strategy is None:
                self.match.robots[elected].strategy = strategy
            elif self.match.robots[elected].strategy.name != strategy.name:
                self.match.robots[elected].strategy = strategy
                self.match.robots[elected].start()
            robots.remove(elected)


    def elect_attacker(self, robot):
        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_ball

    def elect_goalkeeper(self, robot):
        dist_to_goal = math.sqrt(
            (robot.x - 0)**2 + (robot.y - 0.65)**2
        )
        return 1000 - dist_to_goal

    def elect_midfielder(self, robot):
        return 1