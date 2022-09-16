import math
import time
import api
import threading
from collections import deque

from controller import PID_control
from algorithms import rrt
from multiprocessing import Process

from strategy.BaseStrategy import Strategy

class OffensivePathPlanning(threading.Thread):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.path = []

        self.iteration = 0
        self.last_state = [0, 0]

    def run(self):
        while True:
            state = [self.match.ball.x, self.match.ball.y]
            
            self.last_state = state

            obstacle_list = [ [r.x, r.y, r.dimensions['L']] for r in self.match.opposites]

            field_size = self.match.game.field.get_dimensions()

            rrt_ = rrt.RRT(
                start=[self.robot.x, self.robot.y],
                goal=[self.match.ball.x, self.match.ball.y],
                rand_area=[0, field_size[0]],
                obstacle_list=obstacle_list,
                play_area=[0, field_size[0], 0, field_size[1]],
                path_resolution=0.005,
                expand_dis=0.025,
                robot_radius=self.robot.dimensions['L']/2,
                max_iter=1000,
            )

            self.path = rrt_.planning(False)[::-1]
            time.sleep(1/30)
            self.iteration += 1
            



        


class Attacker(Strategy):
    def __init__(self, match, name = 'PID-Attacker'):
        super().__init__(match,
            name=name,
            controller=PID_control
        )

        self.planning = None
        self.rrt_iteration = -1
        self.path = []

    def start(self, robot=None):
        super().start(robot=robot)

        self.planning = OffensivePathPlanning(self.match, robot)
        self.rrt_iteration = -1

        self.planning.start()


    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)
    
    def next_point(self):
        if self.planning.iteration > self.rrt_iteration:
            self.path = deque(self.planning.path)
            self.rrt_iteration = self.planning.iteration
        
        if len(self.path) == 0:
            return self.robot.x, self.robot.y 

        return self.path[min(3, len(self.path)-1)]


    def decide(self):
        api.Api().send_custom_data(
            {
                'circuit': list(self.planning.path),
                'robot': {'x': self.robot.x, 'y': self.robot.y},
                'obstacles': [{'x': r.x, 'y': r.y} for r in self.match.opposites]
            }
        )
        return self.next_point()
