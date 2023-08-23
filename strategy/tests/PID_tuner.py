from collections import deque
from controller.PID_control import PID_W_control
from strategy.BaseStrategy import Strategy
from api import Api, Api_recv
import time
import math


class PIDTuner(Strategy):
    def __init__(self, match):
        super().__init__(match, "PID_Tune", controller=PID_W_control,
                         controller_kwargs={'V_MAX': 0, 'V_MIN': 50, 'KP': -3, 'KI': 0, 'KD': 0})

        self.sender = Api("127.0.0.1", 43210)
        self.sender.start()
        self.reciver = Api_recv(match, "127.0.0.1", 43221)
        self.reciver.start()
        self.circuit = []
        self.t0 = time.time()
        self.state = "countdown"
        self.still = True
        print("on init")

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

    def make_hexagonal_path(self):
        if .15 <= self.robot.x <= .3 and .575 <= self.robot.y <= .725:

            # set points on the hexagonal path that the robot will perform
            A = (self.robot.x, self.robot.y)
            B = (self.robot.x + .425, self.robot.y + .425)
            C = (self.robot.x + .625, self.robot.y + .425)
            D = (self.robot.x + 1.05, self.robot.y)
            E = (self.robot.x + .625, self.robot.y - .425)
            F = (self.robot.x + .425, self.robot.y - .425)

            # set the circuit
            self.circuit = [B, C, D, E, F, A]

    def make_line_path(self):
        size = .5

        start = self.robot.x, self.robot.y
        mid = self.robot.x + size*math.cos(self.robot.theta), self.robot.y + size*math.sin(self.robot.theta)

        if .15 < mid[0] < 1.35 and .15 < mid[1] < 1.15:
            self.circuit = [mid, start]
        else:
            mid = self.robot.x + size * math.cos(self.robot.theta - math.pi), self.robot.y + size * math.sin(self.robot.theta - math.pi)
            if .15 < mid[0] < 1.35 and .15 < mid[1] < 1.15:
                self.circuit = [mid, start]

    def make_inplace(self):
        self.controller.V_MAX = 0
        self.controller.V_MIN = 0
        self.still = True

        self.circuit = [(self.robot.x + 1, self.robot.y), (self.robot.x, self.robot.y + 1)]

    def decide(self):
        target = self.robot
        data = {'pid': {
                    'set_point': 0,
                    'error': abs(self.controller.error),
                    'time': time.time(),
                    'w': self.robot.vtheta,
                    'running': self.state == "running"},
                'pos':{
                    'x': self.robot.x,
                    'y': self.robot.y,
                    'theta': self.robot.theta
                }
        }

        self.sender.send_custom_data(data)

        data = self.reciver.decod_data

        if data:
            if self.controller.KP != data['kp']:
                print(self.controller.KP, data['kp'])
                self.controller.KP = data['kp']
                self.state = "position"
            if self.controller.KI != data['ki']:
                print(self.controller.KI, data['ki'])
                self.controller.KI = data['ki']
                self.state = "position"
            if self.controller.KD != data['kd']:
                print(self.controller.KI, data['ki'])
                self.controller.KD = data['kd']
                self.state = "position"

        if self.state == "position":
            self.controller.alpha_old = 0
            self.controller.int_alpha = 0

            self.make_inplace()
            print("position")
            if len(self.circuit) > 0:
                print(self.circuit)
                self.state = "countdown"
                self.t0 = time.time()

        if self.state == "countdown":
            if time.time() - self.t0 <= 0: # 3s
                print("countdown")
                target = self.robot
            else:
                self.make_inplace()
                self.state = "running"
                self.t0 = time.time()

        if self.state == "running":
            print(self.circuit)
            if not self.circuit:
                self.state = "wait"

            elif self.still:
                target = self.circuit[0]

                if time.time() - self.t0 > 1:
                    print("reached")
                    self.t0 = time.time()
                    self.circuit.pop(0)
            else:
                target = self.circuit[0]

                dx = self.circuit[0][0] - self.robot.x
                dy = self.circuit[0][1] - self.robot.y

                if math.sqrt(dx ** 2 + dy ** 2) < 0.05:
                    print("reached")
                    self.circuit.pop(0)

                self.controller.V_MAX = 50

        if self.state == "wait":
            target = self.robot
            self.controller.V_MAX = 0

        return target
