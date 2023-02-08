import math
from collections import deque
from strategy.BaseStrategy import Strategy
from controller import PID_control, PID_W_control, SimpleLQR, TwoSidesLQR, UniController
from algorithms.limit_cycle import LimitCycle

class LimitCycle_Test(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, name='LimitCycle_Test', controller=PID_W_control)

        self.circuit = [(0.375, 0.25), (1.125, 0.25), (0.75, 0.65), (1.125, 1.05), (0.375, 1.05)]
        self.circuit = deque(self.circuit)
        self.dl = 0.000001

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def next_point(self):
        point = self.circuit[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.circuit.rotate(-1)
            print("Change point! ", self.circuit[0])

        return self.circuit[0]

    def get_virtual_obstacle(self, target):
        '''
        - m:    angle of the line perpendicular to the line between the ball and
                the center of the goal
        - p:    distance of the virtual obstacles to the ball / radius of the virtual obstacle
        - vo:   virtual obstacle
        - j:    this is the angle between the ball and the center of the goal
        - m:    the normal angle perpendicular to j
        - r:    radius of the ball
        '''
        aim_point = [self.field_h/2, self.field_w]

        j = math.atan2(aim_point[0] - target[1], aim_point[1] - target[0])
        m = j + math.pi/2
        p = 0.1

        r =  .02 #(.0427)/2

        '''
        the terms r*cos(j) and r*sin(j) are subtracted to move
        the center of the obstacles behind the ball instead of its center
        '''
        if self.robot.y < math.tan(j)*(self.robot.x - target[0]) + target[1]:
            virtual_obstacle = (target[0] - p*math.cos(m) - r*math.cos(j), target[1] - p*math.sin(m) - r*math.sin(j), p, 1)
        else:
            virtual_obstacle = (target[0] + p*math.cos(m) - r*math.cos(j), target[1] + p*math.sin(m) - r*math.sin(j), p, -1)
        
        return virtual_obstacle

    def start(self, robot=None):
        super().start(robot=robot)

        self.limit_cycle = LimitCycle(self.match)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        target = (self.match.ball.x, self.match.ball.y)
        virtual_obstacle = self.get_virtual_obstacle(target)

        self.limit_cycle.set_target(target)
        self.limit_cycle.add_obstacle(virtual_obstacle)

        return self.limit_cycle.compute(self.robot)
