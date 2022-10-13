from entities.coach.coach import BaseCoach

import strategy

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "IRON_2022"
    def __init__(self, match):
        super().__init__(match) # chamada do metodo da classe mae

        # vamos usar strategies de teste por enquanto, essa deixa o robo parado
        self._9 = strategy.iron2022.Goalkeeper(self.match)
        self._3 = strategy.tests.Idle(self.match)
        self._0 = strategy.tests.Idle(self.match)

    def decide(self):
        # esta lista eh ordenada em [robot_0, ..., robot_n]
        robots = [(i, r.robot_id) for i, r in enumerate(self.match.robots)]
        strategies = [
            self._3,
            self._9,
            self._0
        ]
        for robot, strategy in zip(robots, strategies):
            if self.match.robots[robot[0]].strategy is not None:
            # vamos evitar chamar o start todo frame
            # pode ser que essa strategia seja pesada de carregar
                continue
            self.match.robots[robot[0]].strategy = strategy
            self.match.robots[robot[0]].start()