from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy.larc2023


class Coach(BaseCoach):
    NAME = "LARC_2023_G1"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.larc2023.ShadowAttacker(self.match)
        self.ST_strategy = strategy.larc2023.MainStriker(self.match)
        self.GK_strategy = strategy.larc2023.Goalkeeper_Spin(self.match) #Goalkeeper_1, doesn't change goalkeeper, spins when ball is close
        self.GK_id = 0  # Goalkeeper fixed ID

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

    def decide(self):
        GK = [i for i, r in enumerate(self.match.robots) if r.robot_id is self.GK_id][0]
        strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK_id]
        print(GK)
        print(strikers)
        ST, SS = self.choose_main_striker(*strikers)

        st_strat, ss_start = self.handle_stuck(ST, SS)

        if self.match.robots[GK].strategy is None:
            self.match.robots[GK].strategy = self.GK_strategy
            self.match.robots[GK].start()
        else:
            if self.match.robots[GK].strategy.name != self.GK_strategy:
                self.match.robots[GK].strategy = self.GK_strategy
                self.match.robots[GK].start()

        ST.strategy = st_strat
        ST.start()

        SS.strategy = ss_start
        SS.start()

    def choose_main_striker(self, r1, r2):
        b = self.match.ball

        a1 = distance_between_points((b.x, b.y), (r1.x, r1.y))
        a2 = distance_between_points((b.x, b.y), (r2.x, r2.y))

        b1, b2 = b.x - r1.x, b.x - r2.x

        if b1 * b2 > 0:
            if a1 < a2:
                return r1, r2
            return r2, r1
        if b1 > 0:
            return r1, r2
        return r2, r1

    def choose_gk(self, gk,  r1, r2):
        
        dist_r1 = distance_between_points((0, 0.65), (r1.x, r1.y))
        dist_r2 = distance_between_points((0, 0.65), (r2.x, r2.y))

        gk_down, r1_down, r2_down = gk.y - 0.65, r1.y - 0.65, r2.y - 0.65,

        if gk_down * r1_down < 0:
            return r2, r1, gk
        if gk_down * r2_down < 0:
            return r1, gk, r2
        if dist_r1 > dist_r2:
            return r2, r1, gk
        return r1, gk, r2        
    
    def check_change_gk(self, gk, x_attack = 0.4):
        ball = self.match.ball

        dist_bgk = distance_between_points((ball.x, ball.y), (gk.x, gk.y))


        if gk.x > 0.4 and dist_bgk < 0.1:
            return True
        return False
            
    def handle_stuck(self, ST, SS):
        game_runing = not (self.match.game_status == 'STOP' or self.match.game_status == None)
        stuck_st = ST.is_stuck() and game_runing
        stuck_ss = SS.is_stuck() and game_runing

        # if stuck_st and stuck_ss:
        #     return self.unstucks[ST.robot_id], self.unstucks[SS.robot_id]
        #
        # if stuck_st and not stuck_ss:
        #     return self.unstucks[ST.robot_id], self.ST_strategy
        #
        # if not stuck_st and stuck_ss:
        #     return self.ST_strategy, self.unstucks[SS.robot_id]

        return self.ST_strategy, self.SS_strategy
