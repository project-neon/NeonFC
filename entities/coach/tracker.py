from entities.coach.coach import BaseCoach
from math import sin, cos
import strategy
import time

class Coach(BaseCoach): # heranca da classe abstrata
    NAME = "TRACKER"
    def __init__(self, match):
        super().__init__(match) # chamada do metodo da classe mae

        self.v, self.w = 70, 0

        # vamos usar strategies de teste por enquanto, essa deixa o robo parado
        self.I1 = strategy.tests.Idle(self.match)
        self.I2 = strategy.tests.Idle(self.match)
        self.Moving = strategy.tests.Move(self.match, self.v, self.w)

        self.vxs = ['vx']
        self.vys = ['vy']
        self.xs = ['x']
        self.ys = ['y']
        self.t = ['time']

        self.moving_id = 9

    def decide(self):
        print('d')
        moving = [(i, r) for i, r in enumerate(self.match.robots) if r.robot_id is self.moving_id][0]
        still = [i for i, r in enumerate(self.match.robots) if r.robot_id is not self.moving_id]

        for robot, strategy in zip(still, [self.I1, self.I2]):
            if self.match.robots[robot].strategy is not None:
                continue
            self.match.robots[robot].strategy = strategy
            self.match.robots[robot].start()

        print('going_2_crash', self.going_2_crash(moving[1]))
        if self.going_2_crash(moving[1]):
            with open(f'robot_{self.moving_id}_at_v_{self.v}_w{self.w}.csv', 'w') as file:
                for line in zip(self.vxs, self.vys, self.xs, self.ys, self.t):
                    file.write(";".join(line))
                    file.write("\n")
            self.Moving.stop()
        else:
            if self.match.robots[moving[0]].strategy is None:
                self.match.robots[moving[0]].strategy = self.Moving
                self.match.robots[moving[0]].start()
            self.vxs.append(str(moving[1].vx))
            self.vys.append(str(moving[1].vy))
            self.xs.append(str(moving[1].x))
            self.ys.append(str(moving[1].y))
            self.t.append(str(time.time()))


    def going_2_crash(self, robot):
        a = .01

        x_pred = robot.x + self.v * cos(robot.theta) * a
        y_pred = robot.y + self.v * sin(robot.theta) * a

        if x_pred > 1.5 or x_pred < 0 or y_pred > 1.3 or y_pred < 0:
            return True
        return False
