from entities.coach.coach import BaseCoach

import strategy

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "TEST"
    def __init__(self, match):
        super().__init__(match) # chamada do metodo da classe mae

        # vamos usar strategies de teste por enquanto, essa deixa o robo parado
        self.attacker_strategy = strategy.tests.Idle(self.match)
        self.midfielder_strategy = strategy.tests.Idle(self.match)
        self.goalkeeper_strategy = strategy.iron2023.Goalkeeper(self.match)

    def decide(self):

        self.match.robots[0].strategy = self.goalkeeper_strategy
        self.match.robots[0].start()
        self.match.robots[1].strategy = self.midfielder_strategy
        self.match.robots[1].start()
        self.match.robots[2].strategy = self.attacker_strategy
        self.match.robots[2].start()

        # # esta lista eh ordenada em [robot_0, ..., robot_n]
        # robots = [r.robot_id for r in self.match.robots]
        # strategies = [
        # self.goalkeeper_strategy,
        # self.attacker_strategy, self.midfielder_strategy

        # ]
        # for robot, strategy in zip(robots, strategies):
        #     if self.match.robots[robot].strategy is not None:
        #     # vamos evitar chamar o start todo frame
        #     # pode ser que essa strategia seja pesada de carregar
        #         continue
        #     self.match.robots[robot].strategy = strategy
        #     self.match.robots[robot].start()
