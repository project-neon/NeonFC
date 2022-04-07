import math

import strategy
from entities.plays.playbook import Play


class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self._reset = False
        self.constraints = [
            (strategy.tests.GoalKeeperRCX(self.match, "Goalkeeper", "MidFielderSupporter"), self._elect_goalkeeper),
            (strategy.tests.SpinnerAttacker(self.match), self._elect_attacker),
            (strategy.tests.MidFielderSupporter(self.match, attacker="SpinnerAttacker"), self._elect_midfielder)
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        super().start_up()
        self._reset = True
    
    def freeze_positions(self, constraints, robots):
        constraints = self.constraints[1:]
        robots = [
            r_id for r_id in robots 
            if self.match.robots[r_id].strategy.name not in ("Goalkeeper")
        ]
        return constraints, robots

    def update(self):
        super().update()

        robots = [r.robot_id for r in self.match.robots]
        constraints = self.constraints

        # verify if the game is running
        # if is running don't change goalkeeper or attacker
        if self._can_play() and (not self._reset):
            constraints, robots = self.freeze_positions(
                constraints, robots
            )

        if self._reset == True:
            self._reset = False

        for strategy, fit_fuction in constraints:
            elected, best_fit = -1, -99999

            index = 0
            for robot_id in robots:
                for i in range(len(self.match.robots)):
                    if self.match.robots[i].robot_id == robot_id:
                        index = i
                robot_fit = fit_fuction(self.match.robots[index])
                if (robot_fit > best_fit):
                    best_fit, elected = robot_fit, index
            
            if self.match.robots[elected].strategy is None:
                self.match.robots[elected].strategy = strategy
            elif self.match.robots[elected].strategy.name != strategy.name:
                self.match.robots[elected].strategy = strategy
                self.match.robots[elected].start()

            robots.remove(self.match.robots[elected].robot_id)

    def _elect_attacker(self, robot):
        is_behind = 2 if robot.x > self.match.ball.x else 1
        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_ball * is_behind

    def _elect_goalkeeper(self, robot):
        dist_to_goal = math.sqrt(
            (robot.x - 0)**2 + (robot.y - 0.65)**2
        )
        return 1000 - dist_to_goal

    def _elect_midfielder(self, robot):
        return 1