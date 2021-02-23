import algorithims
import strategy
import math
import time

from commons.math import angular_speed, speed, rotate_via_numpy, unit_vector

class IronCupCoach(object):
    def __init__(self, match):
        self.match = match
        self.constraints = [
            #estratégia - função eleitora - robot_id
            (strategy.larc2020.GoalKeeper(self.match), self.elect_goalkeeper, 0),
            (strategy.larc2020.Attacker(self.match), self.elect_attacker, 0),
            (strategy.larc2020.MidFielder(self.match), self.elect_midfielder, 0)
        ]

        self.avoid_strategy = strategy.iron2021.Avoid(self.match)

        self.attacker = None
        self.midfielder = None
        self.goalkeeper = None
    
    def decide (self):
        robots = [r.robot_id for r in self.match.robots]

        for robot in robots:
            if self.match.robots[robot].is_stuck():
                print('{} is stuck! {}'.format(self.match.robots[robot].get_name(), time.time()))
    
        for strategy, fit_fuction, priority in self.constraints:
            elected = -1
            best_fit = -99999
            for robot_id in robots:
                robot_fit = fit_fuction(self.match.robots[robot_id])
                if (robot_fit > best_fit):
                    best_fit = robot_fit
                    elected = robot_id
            
            priority = elected
            if self.match.robots[elected].strategy is None:
                self.match.robots[elected].strategy = strategy
            elif self.match.robots[elected].strategy.name != strategy.name:
                self.match.robots[elected].strategy = strategy
                self.match.robots[elected].start()
            robots.remove(elected)
        
        robots = [r.robot_id for r in self.match.robots]
        
        dict_robots = {
            self.match.robots[r].strategy.name: self.match.robots[r] for r in robots
        }

        if robots:
            if dict_robots['attacker'].is_stuck() and dict_robots['midfielder'].is_stuck():
                dict_robots['midfielder'].strategy = self.avoid_strategy
                dict_robots['midfielder'].start()

        
        for robot in robots:
            print('{} using: [{}]'.format(self.match.robots[robot].get_name(), self.match.robots[robot].strategy.name))

    def elect_attacker(self, robot):

        is_behind = 2 if robot.x > self.match.ball.x else 1

        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_ball * is_behind

    def elect_goalkeeper(self, robot):
        dist_to_goal = math.sqrt(
            (robot.x - 0)**2 + (robot.y - 0.65)**2
        )
        return 1000 - dist_to_goal

    def elect_midfielder(self, robot):
        return 1

    