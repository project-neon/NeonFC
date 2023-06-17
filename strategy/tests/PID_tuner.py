from collections import deque
from controller.PID_control import PID_W_control
from strategy.BaseStrategy import Strategy
from api import Api, Api_recv
import time
import math


class PIDTuner(Strategy):
    def __init__(self, match):
        super().__init__(match, "PID_Tune", controller=PID_W_control,
                         controller_kwargs={'V_MAX': 0, 'V_MIN': 0, 'KP': 0, 'KI': 0, 'KD': 0})
        self.sender = Api("127.0.0.1", 43210)
        self.sender.start()
        self.reciver = Api_recv(match, "127.0.0.1", 43221)
        self.reciver.start()
        self.circuit = [(.2, .65), (1.1, .65)]  # , (.4, .90), (.4, .40)]
        self.circuit = deque(self.circuit)
        self.t0 = time.time()
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

    def decide(self):
        data = {'pid': {
                    'set_point': 0,
                    'error': self.controller.error,
                    'time': time.time(),
                    'w': self.robot.vtheta},
                'pos':{
                    'x': self.robot.x,
                    'y': self.robot.y,
                    'theta': self.robot.theta
                }
        }

        self.sender.send_custom_data(data)

        data = self.reciver.decod_data
        new_ks = False

        if data:
            if self.controller.KP != data['kp']:
                print(self.controller.KP, data['kp'])
                self.controller.KP = data['kp']
                new_ks = True
            if self.controller.KI != data['ki']:
                print(self.controller.KI, data['ki'])
                self.controller.KI = data['ki']
                new_ks = True
            if self.controller.KD != data['kd']:
                print(self.controller.KI, data['ki'])
                self.controller.KD = data['kd']
                new_ks = True

        if new_ks:
            print("new ks")
            self.controller.alpha_old = 0
            self.controller.int_alpha = 0
            new_ks = False

            self.t0 = time.time()

        if time.time() - self.t0 <= 1:
            target = [.5 * self.robot.x, .5 * self.robot.y]

        elif time.time() - self.t0 <= 2:
            target = [self.robot.x + math.cos(self.robot.theta + .5 * math.pi),
                      self.robot.y + math.sin(self.robot.theta + .5 * math.pi)]
            self.lp = [self.robot.x, self.robot.y, self.robot.theta]

        elif time.time() - self.t0 <= 3:
            target = [self.lp[0] + math.cos(self.lp[2] + .5 * math.pi),
                      self.lp[1] + math.sin(self.lp[2] + .5 * math.pi)]

        else:
            # self.controller.KP, self.controller.KI, self.controller.KD = 0, 0, 0
            target = self.robot


        return target
