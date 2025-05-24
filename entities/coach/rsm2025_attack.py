from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy
import json

import strategy.rsm2025
import strategy.rsm2025


class Coach(BaseCoach):
    NAME = "RSM_2025_Attack"

    def __init__(self, match):
        super().__init__(match)
        self.SS_strategy = strategy.rsm2025.ShadowAttacker(self.match)
        self.ST_strategy = strategy.rsm2025.MainStriker(self.match)
        self.GK_strategy = strategy.rsm2025.Goalkeeper(self.match)

        self.GK_id = 5  # Goalkeeper fixed ID
        self.defending = False

        self.ball = self.match.ball
        positions = json.loads(open('foul_placements3v3.json', 'r').read())
        self._position = {r.robot_id: strategy.commons.Replacer(self.match, positions[str(r.robot_id)]) for r in self.match.robots}

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

        self.GK = next(filter(lambda r: r.robot_id == self.GK_id, self.match.robots))        
        strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id != self.GK_id]
        self.ST, self.SS = self.choose_main_striker(*strikers) 
    started = False

    def start(self):
        self.set_strategy(self.GK, self.GK_strategy)
        self.set_strategy(self.ST, self.ST_strategy)
        self.set_strategy(self.SS, self.SS_strategy)
        self.started = True
        self.ball.x, self.ball.y = 1.5 , 1.3

    def decide(self):
        if not self.started:
            self.start()
            print(self.ST.robot_id, self.SS.robot_id)

        if self.match.match_event['event'] == 'PLAYING':
            cond_strikers = self.strikers_behind([self.ST, self.SS])


            if self.ball.y < 0.3 and self.ball.x < 0.4 and cond_strikers == True:
                self.ST, self.GK, self.SS = self.choose_goalkeeper(self.GK, self.ST, self.SS)
        
                self.set_strategy(self.ST, self.ST_strategy)
                self.set_strategy(self.GK, self.GK_strategy)
                self.set_strategy(self.SS, self.SS_strategy)

            if self.ball.y > 1 and self.ball.x < 0.4 and cond_strikers == True:
                self.ST, self.GK, self.SS = self.choose_goalkeeper(self.GK, self.ST, self.SS)
        
                self.set_strategy(self.ST, self.ST_strategy)
                self.set_strategy(self.GK, self.GK_strategy)
                self.set_strategy(self.SS, self.SS_strategy)

        else:
            self.not_playing()

    def attack(self):
        strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id != self.GK_id]
        self.ST, self.SS = self.choose_main_striker(*strikers)

        self.set_strategy(self.ST, self.ST_strategy)
        self.set_strategy(self.SS, self.SS_strategy)

    def not_playing(self):
        robots = [(i, r.robot_id) for i, r in enumerate(self.match.robots)]
        for robot, strategy in zip(robots, self._position):
            if self.match.robots[robot[0]].strategy == strategy:
                continue
            self.match.robots[robot[0]].strategy = strategy
            self.match.robots[robot[0]].start()

    def set_strategy(self, robot, strat):
        if robot != strat:
            robot.strategy = strat
            robot.start()

    def choose_main_striker(self, r1, r2):
        b = self.ball

        a1 = distance_between_points((b.x, b.y), (r1.x, r1.y))
        a2 = distance_between_points((b.x, b.y), (r2.x, r2.y))

        b1, b2 = b.x - r1.x - 0.05, b.x - r2.x - 0.05

        if b1 * b2 > 0:
            if a1 < a2:
                return r1, r2
            return r2, r1
        if b1 > 0:
            return r1, r2
        return r2, r1
    
    def choose_goalkeeper(self, gk, r1, r2):
        b = self.ball

        a1 = distance_between_points((b.x, b.y), (r1.x, r1.y))
        a2 = distance_between_points((b.x, b.y), (r2.x, r2.y))

        b1, b2 = b.x - r1.x - 0.05, b.x - r2.x - 0.05

        if b1 * b2 > 0:
            if a1 < a2:
                return gk, r1, r2
            return gk, r2, r1
        if b1 > 0:
            return gk, r1, r2
        return gk, r2, r1
    
    def strikers_behind(self, strikers):
        b = self.ball
        r1,r2 = strikers[0], strikers[1] 

        b1, b2 = b.x - r1.x - 0.1, b.x - r2.x - 0.1

        if b1 < 0 and b2 < 0:
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
