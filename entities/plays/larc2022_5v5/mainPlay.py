import strategy
from entities.plays.playbook import Play

class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self.strategies = [
            strategy.larc2022_5v5.GoalKeeper(self.match, 'Goalkeeper'),
            strategy.larc2022_5v5.RadialDefender(self.match, 'Defender1'),
            strategy.larc2022_5v5.RadialDefender(self.match, 'Defender2'),
            strategy.larc2022_5v5.SecondAttacker(self.match, 'SecondAttacker'),
            strategy.larc2022_5v5.MainAttacker(self.match, 'MainAttacker'),
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        super().start_up()

    def update(self):
        super().update()

        for robot, strategy in zip(self.match.robots, self.strategies):
            if robot.strategy is None:
                robot.strategy = strategy
                robot.start()
            elif robot.strategy.name != strategy.name:
                robot.strategy = strategy
                robot.start()


            
