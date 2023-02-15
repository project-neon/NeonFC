from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy


class Coach(BaseCoach):
    NAME = "IRON_2023"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.iron2022.Midfielder(self.match)
        self.ST_strategy = strategy.iron2022.Attacker_LC(self.match)

        self.GK_strategy = strategy.iron2022.Goalkeeper(self.match)
        self.GK_id = 0  # Goalkeeper fixed ID

    def decide(self):
        GK = [i for i, r in enumerate(self.match.robots) if r.robot_id is self.GK_id][0]
        strikers = [(i, r) for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK_id]
        ST = self.choose_main_striker(*strikers)

        self.match.robots[GK].strategy = self.GK_strategy
        self.match.robots[GK].start()

        for robot in strikers:
            if robot[0] == ST:
                self.match.robots[robot[0]].strategy = self.ST_strategy
                self.match.robots[robot[0]].start()

            else:
                self.match.robots[robot[0]].strategy = self.SS_strategy
                self.match.robots[robot[0]].start()

    def choose_main_striker(self, r1, r2):

        b_x, b_y = self.match.ball.x, self.match.ball.y

        a1     = distance_between_points((b_x, b_y), (r1[1].x, r1[1].y))
        a2     = distance_between_points((b_x, b_y), (r2[1].x, r2[1].y))
        b1, b2 = r1[1].y - self.match.bally, r2[1].y - self.match.bally

        if b1 * b2 > 0:
            if a1 < a2:
                return r1[0]
            return r2[0]
        if b1 < 0:
            return r1[0]
        return r2[0]
