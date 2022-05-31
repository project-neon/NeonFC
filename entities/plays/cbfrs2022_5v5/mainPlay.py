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
            (strategy.cbfrs2022_5v5.GoalKeeper(self.match), self._elect_goalkeeper),
            (strategy.tests.UVFAttacker(self.match, self.coach), self._elect_leftattacker),
            (strategy.cbfrs2022_5v5.LeftWing(self.match), self._elect_leftwing),
            (strategy.cbfrs2022_5v5.RightWing(self.match), self._elect_rightwing),
            (strategy.cbfrs2022_5v5.RightAttacker(self.match), self._elect_rightattacker),
        ]

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def _can_play(self):
        return self.match.game.referee.can_play()

    def start_up(self):
        super().start_up()
        self._reset = True
    
    def freeze_positions(self, constraints, robots):
        constraints = self.constraints[3:]
        robots = [
            r_id for r_id in robots 
            if self.match.robots[r_id].strategy.name not in (
                "Goalkeeper", "LeftWing", "RightWing", "RightAttacker", "UFV-Attacker"
            )
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

        if self.match.ball.x > self.field_w-0.35 and self.field_h/2-0.4 < self.match.ball.y < self.field_h/2+0.4:
            self.freeze_positions(constraints, robots)
            print("froze")
            return

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

        self.coach.ball_dists = {
                f"{robot.robot_id}": math.sqrt(
                    (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
                ) for robot in self.match.robots if not robot.strategy.name == "Goalkeeper"
            }

    def _elect_goalkeeper(self, robot):
        dist_to_goal = math.sqrt(
            (robot.x - 0)**2 + (robot.y - self.field_h/2)**2
        )
        return 1000 - dist_to_goal

    def _elect_leftattacker(self, robot):
        is_behind = 2 if robot.x > self.match.ball.x else 1
        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_ball * is_behind
        

    def _elect_rightattacker(self, robot):
        dist_to_target = math.sqrt(
            (robot.x - self.field_w/2 - 0.3)**2 + (robot.y - self.field_h/2 - 0.15)**2
        )
        
        return 1

    def _elect_leftwing(self, robot):
        dist_to_2q = self.field_h - robot.y
        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        
        return 1000 - dist_to_2q

    def _elect_rightwing(self, robot):
        dist_to_3q = robot.y
        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_3q