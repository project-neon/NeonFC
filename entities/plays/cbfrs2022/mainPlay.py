import math

import strategy
from entities.plays.playbook import Play


class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self.strategies = [
            strategy.cbfr2022.MainGoalkeeper(self.match),
            strategy.cbfr2022.MainAttacker(self.match),
            strategy.cbfr2022.MainMidFielder(self.match)
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        super().start_up()

    def update(self):
        super().update()

        robots = [r.robot_id for r in self.match.robots]


        for robot, strategy in zip(robots, self.strategies):
            if self.match.robots[robot].strategy is not None:
                continue
            self.match.robots[robot].strategy = strategy
            self.match.robots[robot].start()
