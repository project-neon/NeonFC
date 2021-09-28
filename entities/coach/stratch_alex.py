from commons.math import distance_to_line
import json
import math

import strategy
from entities.coach.coach import BaseCoach

class Coach(BaseCoach):
    NAME = "DEV_ALEX"
    def __init__(self, match):
        super().__init__(match)

        self.constraints = [
            #estratégia - função eleitora - prioridade
            (strategy.larc2020.GoalKeeper(self.match), self.elect_goalkeeper, 0),
            (strategy.tests.UVFAttacker(self.match), self.elect_attacker), 0),
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

    def get_range_of_aim(self, robot):
        goal_ = self.match.game.field.get_dimensions()
        goal_left = [goal_[0], goal_[1]/2 + 0.15]
        goal_right = [goal_[0], goal_[1]/2 - 0.15]
        ball_pos = [self.match.ball.x, self.match.ball.y]

        range_ = []

        for goal in [goal_left, goal_right]:
            ball_to_goal_or = math.atan2(ball_pos[0] - goal[0], ball_pos[1] - goal[1])

            robot_to_goal_or = math.atan2(
                robot.x - goal[0], robot.y - goal[1]
            )

            ball_to_goal_or = math.degrees(ball_to_goal_or)
            if ball_to_goal_or < 0:
                ball_to_goal_or += 90

            robot_to_goal_or = math.degrees(robot_to_goal_or)
            if robot_to_goal_or < 0:
                robot_to_goal_or += 90
            
            range_.append((robot_to_goal_or - ball_to_goal_or))

        return range_
    
    def best_angle(self, robot):
        
        goal = self.match.game.field.get_dimensions()
        goal = [goal[0], goal[1]/2]

        ranges = self.get_range_of_aim(robot)

        distance_from_aim = distance_to_line(
            self.match.ball.x, self.match.ball.y,
            math.sin(robot.theta), math.cos(robot.theta),
            robot.x, robot.y
        )

        dist_robot_ball = (
            (robot.x - self.match.ball.x)**2
            + (robot.y - self.match.ball.y)**2
        )**.5

        match_rank = 0

        if self.match.ball.x < robot.x:
            match_rank += 100

        if not (ranges[0] > 0 and ranges[1] < 0):
            match_rank += 100
        
        print(robot.get_name(),  match_rank + dist_robot_ball * 10 + distance_from_aim * 5)
        return match_rank * 100 + dist_robot_ball * 10 + distance_from_aim * 100
