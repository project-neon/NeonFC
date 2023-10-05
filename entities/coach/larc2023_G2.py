from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy.larc2023_G2


class Coach(BaseCoach):
    NAME = "LARC_2023_G2"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.larc2023_G2.ShadowAttacker(self.match)
        self.ST_strategy = strategy.larc2023_G2.MainStriker(self.match)
        self.GK_strategy = strategy.larc2023_G2.Goalkeeper(self.match) #Goalkeeper_2, changes goalkeeper, doesn't spins when ball is close
        self.GK_id  = 7
        self.strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK_id]
        self.opposites = self.match.opposites

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

    def decide(self):
        GK, strikers = self.GK, self.strikers
        GK, ST, SS = self.make_choices(GK, *strikers) 

        st_strat, ss_start = self.handle_stuck(ST, SS)
        print('GK, ST, SS:', GK.robot_id, ST.robot_id, SS.robot_id)
        print('self GK ST SS:', self.GK.robot_id, self.ST.robot_id, self.SS.robot_id)
        self.GK, self.strikers = GK, strikers

        st_strat, ss_start = self.handle_stuck(ST, SS)

        if GK.strategy != self.GK_strategy:
            GK.strategy = self.GK_strategy
            GK.start()
        GK.start()

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

    def choose_gk(self, r1, r2):

        dist_br1 = distance_between_points((0, 0.65), (r1.x, r1.y))
        dist_br2 = distance_between_points((0, 0.65), (r2.x, r2.y))

        #gk_r1, gk_r2 = ((gk.x - 0.65)*(r1.x - 0.65)), ((gk.x - 0.65)*(r2.x - 0.65))

        if dist_br1 < dist_br2:
            return r1
        return r2 
        
    def check_change_gk(self, gk, x_attack = 0.4):
        ball = self.match.ball

        dist_bgk = distance_between_points((ball.x, ball.y), (gk.x, gk.y))

        for r in self.opposites:
            if ball.vx < 0 and ball.x < 0.75:
                return True
        return False



    def make_choices(self, gk, r1, r2):
        ball = self.match.ball
        GK, ST, SS = 0, 0, 0


        if self.check_change_gk(gk):
            GK = self.choose_gk(r1, r2) 
        else:
            GK = gk

        not_gk = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK.robot_id]
        
        ST, SS = self.choose_main_striker(*not_gk)

        return GK, ST, SS 

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
        for i in self.match.robots:
            if i.robot_id == 7:
                return i
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


        return (cont-1), gk_way

    
            
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

    #def same_strat(self, r1, r2, gk):
    #    if


