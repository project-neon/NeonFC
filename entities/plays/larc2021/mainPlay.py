import math

import strategy
from entities.plays.playbook import Play


class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.constraints = [
            (strategy.tests.newGoalKeeper(self.match, "Goalkeeper"), self._elect_goalkeeper),
            (strategy.tests.UVFAttacker(self.match), self._elect_attacker),
            (strategy.tests.newMidFielder(self.match), self._elect_midfielder),
            # (strategy.tests.Defender(self.match, "Left"), self._elect_midfielder),
            # (strategy.tests.Defender(self.match, "Right"), self._elect_midfielder),
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def update(self):
        super().update()

        robots = [r.robot_id for r in self.match.robots]
        constraints = self.constraints

        # verify if the game is running
        # if is running don't change goalkeeper or attacker
        if self._can_play():
            constraints = self.constraints[1:]
            robots = [
                r_id for r_id in robots 
                if self.match.robots[r_id].strategy.name not in ("Goalkeeper", "LeftDefender", "RightDefender")
            ]

        for strategy, fit_fuction in constraints:
            elected, best_fit = -1, -99999

            for robot_id in robots:
                robot_fit = fit_fuction(self.match.robots[robot_id])
                if (robot_fit > best_fit):
                    best_fit, elected = robot_fit, robot_id
            
            if self.match.robots[elected].strategy is None:
                self.match.robots[elected].strategy = strategy
            elif self.match.robots[elected].strategy.name != strategy.name:
                self.match.robots[elected].strategy = strategy
                self.match.robots[elected].start()

            robots.remove(elected)

    def _elect_attacker(self, robot):

        is_behind = 2 if robot.x > self.match.ball.x else 1

        x_ball = self.match.ball.x + self.match.ball.vx * (1/60)
        y_ball = self.match.ball.y + self.match.ball.vy * (1/60)

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