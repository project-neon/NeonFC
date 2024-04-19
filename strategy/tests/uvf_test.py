import math
import time
from collections import deque
from strategy.BaseStrategy import Strategy
from controller import PID_control, SimpleLQR, TwoSidesLQR, UniController
from NeonPathPlanning import UnivectorField

class UVF_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, name="UVF_Test", controller=UniController)

        self.circuit = [(0.375, 0.25), (1.125, 0.25), (0.75, 0.65), (1.125, 1.05), (0.375, 1.05)]
        self.circuit = deque(self.circuit)
        self.dl = 0.000001

    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]

    def start(self, robot=None):
        super().start(robot=robot)

        self.Univector = UnivectorField(n=6, rect_size=.375)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):

        ball_x, ball_y = self.match.ball.x, self.match.ball.y
        theta_ball = math.atan2(0.65 - ball_y,1.6 - ball_x)
        ball_rx, ball_ry = ball_x + .05*math.cos(theta_ball), ball_y + .05*math.sin(theta_ball)

        self.Univector.set_target(g=(ball_x, ball_y), r=(ball_rx, ball_ry))

        x, y = self.robot.x, self.robot.y

        theta_d = self.Univector.compute((x, y))
        theta_f = self.Univector.compute((x + self.dl*math.cos(self.robot.theta), y + self.dl*math.sin(self.robot.theta)))

        return theta_d, theta_f
