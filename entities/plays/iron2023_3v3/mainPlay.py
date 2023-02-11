import math
import strategy
from entities.plays.playbook import Play

class MainPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.coach = coach
        self.strategies = [
            strategy.larc2022_5v5.GoalKeeper(self.match, 'Goalkeeper'),
            strategy.iron2023_3v3.Midfielder(self.match, 'Midfielder'),
            strategy.iron2023_3v3.MainAttacker(self.match, 'MainAttacker'),
        ]

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        super().start_up()

    def update(self):
        super().update()
        
        attackers = self.match.robots[-2:]
        best_attacker = None
        best_fit = -9999
        for attacker_candidate in attackers:
            fit = self._elect_leftattacker(attacker_candidate)
            if fit > best_fit:
                best_attacker = attacker_candidate.robot_id
                best_fit = fit
        
        if best_attacker == 1:
            strategies = self.strategies[0:1] + [self.strategies[2], self.strategies[1]]
        else:
            strategies = self.strategies


        for robot, strategy in zip(self.match.robots, strategies):
            if robot.strategy is None:
                robot.strategy = strategy
                robot.start()
            elif robot.strategy.name != strategy.name:
                robot.strategy = strategy
                robot.start()


            
    def _elect_leftattacker(self, robot):
        is_behind = 4 if robot.x > self.match.ball.x else 1
        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        is_1 = 0
        if robot.robot_id == 2:
            is_1 = 10000

        return is_1