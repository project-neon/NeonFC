import strategy

from entities.coach.coach import BaseCoach


class Coach(BaseCoach):  # heranca da classe abstrata
    NAME = "TEST"

    def __init__(self, match):
        super().__init__(match)  # chamada do metodo da classe mae

        self._1 = strategy.larc2023.Goalkeeper(self.match)
        self._2 = strategy.tests.Idle(self.match)
        self._3 = strategy.tests.Idle(self.match)

    def decide(self):
        # esta lista eh ordenada em [robot_0, ..., robot_n]
        robots = [(i, r.robot_id) for i, r in enumerate(self.match.robots)]
        strategies = [
            self._1,
            self._2,
            self._3
        ]

        for robot, strategy in zip(robots, strategies):
            if self.match.robots[robot[0]].strategy is not None:
                # vamos evitar chamar o start todo frame
                # pode ser que essa strategia seja pesada de carregar
                continue
            self.match.robots[robot[0]].strategy = strategy
            self.match.robots[robot[0]].start()
