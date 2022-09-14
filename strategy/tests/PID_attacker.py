import math
import api
from collections import deque

from strategy.BaseStrategy import Strategy
from controller import PID_control


class Attacker(Strategy):
    def __init__(self, match, name = 'PID-Attacker'):
        super().__init__(match,
            name=name,
            controller=PID_control
        )

        self.circuit = [(0, 0)]

    def start(self, robot=None):
        super().start(robot=robot)


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def set_circuit(self, circuit):
        self.circuit = deque(circuit)
    
    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y
    
        if math.sqrt(dx**2 + dy**2) < 0.025:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]


    def decide(self):
        api.Api().send_custom_data(
            {
                'circuit': list(self.circuit),
                'robot': {'x': self.robot.x, 'y': self.robot.y, 'theta': self.robot.theta},
                'speed': (self.robot.vx**2 + self.robot.vy**2)**.5
            }
        )
        return self.next_point()
