from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy.larc2023


class Coach(BaseCoach):
    NAME = "LARC_2023_G2"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.larc2023.ShadowAttacker(self.match)
        self.ST_strategy = strategy.larc2023.MainStriker(self.match)
        self.GK_strategy = strategy.larc2023.Goalkeeper(self.match) #Goalkeeper_2, changes goalkeeper, doesn't spins when ball is close
        self.GK  = self.distance_goal()
        self.strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK.robot_id]
        self.opposites = self.match.opposites

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

    def decide(self):
        GK = self.GK
        strikers = self.strikers
        ST, SS = self.choose_main_striker(*strikers)

        st_strat, ss_start = self.handle_stuck(ST, SS)

        if self.check_change_gk(GK, 0.22):
            GK, strikers[0], strikers[1] = self.choose_gk(GK, *strikers)

        if GK.strategy is None:
            GK.strategy = self.GK_strategy
            GK.start()
        else:
            if GK.strategy.name != self.GK_strategy:
                GK.strategy = self.GK_strategy
                GK.start()

        ST.strategy = st_strat
        ST.start()

        SS.strategy = ss_start
        SS.start()

        self.GK = GK
        self.strikers = strikers
        print("GK, ST, SS:", GK.robot_id, ST.robot_id, SS.robot_id)

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

    def choose_gk(self, gk, r1, r2):

        cont_r1 = 0
        cont_r2 = 0
    
        if distance_between_points((r1.x, r1.y), (0, 0.65)) > distance_between_points((r2.x, r2.y), (0, 0.65)):
            cont_r2 += 1
        cont_r1 += 1

        if r1.x > 0.75:
            cont_r2 += 1
        if r2.x > 0.75:
            cont_r1 += 1

        r1_way, r1_gk = self.robots_on_way(r1)
        r2_way, r2_gk = self.robots_on_way(r2)

        if r1_way >= r2_way:
            if not r2_gk:
                cont_r2 += 1
        if r1_way <= r2_way:
            if not r1_gk:
                cont_r1 += 1

        if cont_r1 > cont_r2:
            return r1, gk, r2
        
        if cont_r1 == cont_r2:
            if r1_way > r2_way:
                return r2, gk, r1
            return r1, gk, r2
        
        return r2, gk, r1
    
    def check_change_gk(self, gk, x_attack = 0.4):
        ball = self.match.ball

        dist_bgk = distance_between_points((ball.x, ball.y), (gk.x, gk.y))

        for r in self.opposites:
            dist_bop = distance_between_points((ball.x, ball.y), (r.x, r.y))
            if gk.x > x_attack and dist_bgk < 0.1 and dist_bop > dist_bgk:
                return True
        return False
    
    def distance_goal(self):

        closest = 0
        dist = 1000

        for r in self.match.robots:
            dist_r = distance_between_points([r.x, r.y], [0, 0.65])
            print(dist_r)
            if dist_r < dist:
                print("Distancia",  r.robot_id, dist_r)
                dist = dist_r
                closest = r
        #for i in self.match.robots:
        #    if i.robot_id == 5:
        #        return i
        return r
    
    def robots_on_way(self, r1, check_gk = True):

        cont  = 0
        gk_way = False

        for r in self.match.robots:
            if (r1.y - 0.75)*(r.y - 0.75) > 0:
                if r.robot_id == self.GK.robot_id and check_gk:
                    gk_way = True
                cont += 1
        
        for r in self.match.opposites:
            if (r1.y -0.75)*(r.y - 0.75) > 0:
                cont += 1


        return [(cont-1), gk_way]

    
            
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
