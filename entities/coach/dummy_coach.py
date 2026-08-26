import json

import strategy.dummyStrategy
from entities.coach.coach import BaseCoach
from match.match_real_life import MatchRealLife


class Coach(BaseCoach):
    NAME = "DummyCoach"
    started = False

    def __init__(self, match: MatchRealLife):
        super().__init__(match)
        self.FL_strategy = strategy.dummyStrategy.Follower(self.match)
        self.ball = self.match.ball

        with open("foul_placements3v3.json", "r") as file:
            positions = json.loads().read(file)

        self._position = {
            r.robot_id: strategy.commons.Replacer(
                self.match, positions[str(r.robot_id)]
            )
            for r in self.match.robots
        }

        self.FL_id = 7
        self.FL = next(
            robot
            for _, robot in enumerate(self.match.robots)
            if robot.robot_id == self.FL_id
        )

        self.defending = False

    def start(self):
        self.set_strategy(self.FL, self.FL_strategy)

        self.started = True
        self.ball.x, self.ball.y = 1.5, 1.3

    def decide(self):
        if not self.started:
            self.start()
            print(f"Follower ID: {self.FL.robot_id}")

        if self.match.match_event["event"] != "PLAYING":
            self.not_playing()

    def not_playing(self):
        robots = [(i, r.robot_id) for i, r in enumerate(self.match.robots)]
        for robot, _strategy in zip(robots, self._position):
            if self.match.robots[robot[0]].strategy == _strategy:
                continue

            self.match.robots[robot[0]].strategy = _strategy
            self.match.robots[robot[0]].start()

    def set_strategy(self, robot, strat):
        if robot != strat:
            robot.strategy = strat
            robot.start()
