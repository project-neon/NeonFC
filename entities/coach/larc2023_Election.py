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
        self.GK = self.set_gk()
        self.strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK.robot_id]
        self.opposites = self.match.opposites

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

    def decide(self):
        self.GK, self.strikers = self.GK, self.strikers

        self.GK, self.ST, self.SS = self.make_choices(self.GK, *self.strikers) 

        self.strikers[0], self.strikers[1] = self.ST, self.SS

        st_strat, ss_start = self.handle_stuck(self.ST, self.SS)

        st_strat, ss_start = self.handle_stuck(self.ST, self.SS)

        if self.GK.strategy != self.GK_strategy:
            self.GK.strategy = self.GK_strategy
            self.GK.start()
        self.GK.start()

        self.ST.strategy = st_strat
        self.ST.start()

        self.SS.strategy = ss_start
        self.SS.start()

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

        if distance_between_points((0, 0.65), (r1.x, r1.y)) < distance_between_points((0, 0.65), (r2.x, r2.y)):
            closest = r1
        closest = r2


        r1_way, r1_gk = self.robots_on_way(r1)
        r2_way, r2_gk = self.robots_on_way(r2)

        if closest == r1:
            cont_r1 += 1
        cont_r2 += 1

        if ((gk.x - 0.65)*(r1.x - 0.65)) < 0:
            cont_r1 += 1
        if ((gk.x - 0.65)*(r2.x - 0.65)) < 0:
            cont_r2 += 1
        
        if r1_way < r2_way:
            cont_r1 += 1
        if r1_way > r2_way:
            cont_r2 += 1

        if cont_r1 > cont_r2:
            return r1, gk, r2
        
        if cont_r1 < cont_r2:
            return r2, r1, gk
        
        if r1_gk != r2_gk and r1_gk == True:
            return r2, r1, gk
        if r1_gk != r2_gk and r2_gk == True:
            return r1, gk, r2     

        if closest == r1:
            return r1, gk, r2
        return r2, r1, gk
       
    def check_change_gk(self, gk, r1, r2):
        ball = self.match.ball

        if r1.x >= ball.x and r2.x >= ball.x:
            if ball.x < 1.25 and ball.vx < 0:
                return True
        return False

    def make_choices(self, gk, r1, r2):
        GK, ST, SS = 0, 0, 0
        not_gk = [0,1]

        if self.check_change_gk(gk, r1, r2):
            GK, not_gk[0], not_gk[1] = self.choose_gk(gk, r1, r2) 

        else:
            GK = gk
            not_gk = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK.robot_id]
        
        ST, SS = self.choose_main_striker(*not_gk)

        return GK, ST, SS 

    def set_gk(self):
        for r in self.match.robots:
            if r.robot_id == 0:
                return r
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


