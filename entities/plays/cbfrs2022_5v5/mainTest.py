import strategy
from entities.plays.playbook import Play

class TestPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self.strategies = [
            strategy.cbfrs2022_5v5.RightAttacker(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match),
            strategy.tests.Idle(self.match)
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        super().start_up()

    def update(self):
        super().update()

        self.match.robots[1].strategy = self.strategies[1]
        self.match.robots[2].strategy = self.strategies[2]
        self.match.robots[3].strategy = self.strategies[3]
        self.match.robots[4].strategy = self.strategies[4]

        if self.match.robots[0].strategy is None:
            self.match.robots[0].strategy = self.strategies[0]
            self.match.robots[0].start()
