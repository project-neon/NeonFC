from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy


class Coach(BaseCoach):
    NAME = "RSM_2023"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.rsm2023.ShadowAttacker(self.match)
        self.ST_strategy = strategy.rsm2023.MainStriker(self.match)

        self.GK_strategy = strategy.tests.Idle(self.match)
        self.GK_id = 0  # Goalkeeper fixed ID

        self.unstucks = {r_id: strategy.rsm2023.Unstuck(self.match) for r_id in self.match.robots.robot_ids if r_id != self.GK_id}

    def decide(self):
        GK = [i for i, r in enumerate(self.match.robots) if r.robot_id is self.GK_id][0]
        strikers = [i for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK_id]
        ST, SS = self.choose_main_striker(*strikers)

        st_strat, ss_start = self.handle_stuck(ST, SS)

        self.match.robots[GK].strategy = self.GK_strategy
        self.match.robots[GK].start()

        self.match.robots[ST].strategy = st_strat
        self.match.robots[ST].start()

        self.match.robots[SS].strategy = ss_start
        self.match.robots[SS].start()

    def choose_main_striker(self, r1_idx, r2_idx):
        b = self.match.ball

        r1 = self.match.robots[r1_idx]
        r2 = self.match.robots[r2_idx]

        a1 = distance_between_points((b.x, b.y), (r1.x, r1.y))
        a2 = distance_between_points((b.x, b.y), (r2.x, r2.y))

        b1, b2 = b.x - r1.x, b.x - r2.x

        if b1 * b2 > 0:
            if a1 < a2:
                return r1_idx, r2_idx
            return r2_idx, r1_idx
        if b1 > 0:
            return r1_idx, r2_idx
        return r2_idx, r1_idx

    def handle_stuck(self, ST, SS):
        stuck_st = self.match.robots[ST].is_stuck()
        stuck_ss = self.match.robots[SS].is_stuck()

        if stuck_st and stuck_ss:
            return self.unstucks[ST], self.unstucks[SS]

        if stuck_st and not stuck_ss:
            return self.unstucks[ST], self.ST_strategy

        if not stuck_st and stuck_ss:
            return self.ST_strategy, self.unstucks[SS]

        return self.ST_strategy, self.SS_strategy
