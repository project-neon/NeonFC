from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy
import json

import strategy.rsm2025


class Coach(BaseCoach):
    NAME = "IRON_2025"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.rsm2025.ShadowAttacker(self.match)
        self.ST_strategy = strategy.rsm2025.MainStriker(self.match)
        self.GK_strategy = strategy.rsm2025.Goalkeeper(self.match)
        self.CB_strategy = strategy.rsm2025.Defender(self.match)
        self.SD_strategy = strategy.rsm2025.ShadowDefender(self.match)

        self.GK_id = 0  # Goalkeeper fixed ID
        # todo volta isso pra 5
        self.defending = False

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

    def decide(self):
        if self.match.match_event['event'] == 'PLAYING':
            GK = next(filter(lambda r: r.robot_id == self.GK_id, self.match.robots))
            self.set_strategy(GK, self.GK_strategy)
            
            if self.match.ball.x < .6:
                self.defend()
                # print("Defend")
            else:
                self.attack()
                # print("Attack")

        else:
            self.not_playing()

    def attack(self):
        strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id != self.GK_id]
        ST, SS = self.choose_main_striker(*strikers)

        # print("Attack", ST.robot_id, SS.robot_id)

        self.set_strategy(ST, self.ST_strategy)
        self.set_strategy(SS, self.SS_strategy)

    def defend(self):
        defenders = [r for i, r in enumerate(self.match.robots) if r.robot_id != self.GK_id]
        CB, SD = self.choose_main_defender(*defenders)

        # print("Defend:", CB.robot_id, SD.robot_id)

        self.set_strategy(CB, self.CB_strategy)
        self.set_strategy(SD, self.SD_strategy)

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
        b = self.match.ball

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

    def choose_main_defender(self, r1, r2):
        b = self.match.ball

        a1 = distance_between_points((b.x, b.y), (r1.x, r1.y))
        a2 = distance_between_points((b.x, b.y), (r2.x, r2.y))


        if a1 < a2:
            return r1, r2
        return r2, r1

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
