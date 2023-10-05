from commons.math import distance_between_points
from entities.coach.coach import BaseCoach
import strategy
import json


class Coach(BaseCoach):
    NAME = "LARC_2023_E"

    def __init__(self, match):
        super().__init__(match)

        self.SS_strategy = strategy.rcx2023.ShadowAttacker(self.match)
        self.ST_strategy = strategy.rcx2023.MainStriker(self.match)
        self.GK_strategy = strategy.rcx2023.Goalkeeper(self.match)
        self.GK_id = 5  # Goalkeeper fixed ID
        positions = json.loads(open('foul_placements3v3.json', 'r').read())
        self._position = [strategy.commons.Replacer(self.match, positions[str(r.robot_id)]) for r in self.match.robots]

        # self.unstucks = {r.robot_id: strategy.rsm2023.Unstuck(self.match) for r in self.match.robots if r.robot_id != self.GK_id}

    def decide(self):
        GK = [i for i, r in enumerate(self.match.robots) if r.robot_id is self.GK_id][0]
        strikers = [r for i, r in enumerate(self.match.robots) if r.robot_id is not self.GK_id]
        ST, SS = self.choose_main_striker(*strikers)

        st_strat, ss_start = self.handle_stuck(ST, SS)
        if self.match.match_event['event'] == 'PLAYING':

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

        else:
            robots = [(i, r.robot_id) for i, r in enumerate(self.match.robots)]

            for robot, strategy in zip(robots, self._position):
                if self.match.robots[robot[0]].strategy == strategy:
                    # vamos evitar chamar o start todo frame
                    # pode ser que essa strategia seja pesada de carregar
                    continue
                self.match.robots[robot[0]].strategy = strategy
                self.match.robots[robot[0]].start()

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
