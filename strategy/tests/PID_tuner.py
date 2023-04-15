from collections import deque
from controller.PID_control import PID_W_control
from strategy.BaseStrategy import Strategy
from api import Api, Api_recv
import time
import math


class PIDTuner(Strategy):
    def __init__(self, match):
        self.lt = time.time()
        super().__init__(match, "PID_Tune", controller=PID_W_control)
        self.sender = Api("127.0.0.1", 43210)
        self.sender.start()
        self.reciver = Api_recv(match, "127.0.0.1", 43211)
        self.reciver.start()
        self.circuit = [(.2, .65), (1.1, .65)]  # , (.4, .90), (.4, .40)]
        self.circuit = deque(self.circuit)

    def start(self, robot=None):
        super().start(robot=robot)
        self.y = self.robot.y

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]

    def decide(self):
        data = {'pid': {
            'set_point': 0,
            'error': self.controller.error,
            'time': time.time(),
            'w': self.robot.vtheta
        }}
        self.sender.send_custom_data(data)

        data = self.reciver.decod_data
        if data:
            self.controller.KP = data['kp']
            self.controller.KI = data['ki']
            self.controller.KD = data['kd']

        if self.y == 0:
            self.y = self.robot.y
        target = [.1,self.y]
        print(target)
        return target

        return self.next_point()
