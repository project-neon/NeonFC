import math
import time
from algorithms.RRT.rrt import RRT

import api
import threading
from collections import deque

from controller import PID_control
from algorithms.RRT.rrtstar import RRTStar
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

        self.last_calculation = 0

        self.rrt_ = None

    def run(self):
        while True:
            state = [self.match.ball.x, self.match.ball.y]
            
            self.last_state = state

            obstacle_list = [ [r.x, r.y, r.dimensions['L'] * 1.5/2] for r in self.match.opposites]

            field_size = self.match.game.field.get_dimensions()

            if not self.rrt_ or (
                not self.rrt_.check_path_collision(obstacle_list) or self.last_calculation % 10 == 0
            ):
                print("##### Runs RRT")
                self.rrt_ = RRTStar(
                    start=[self.robot.x, self.robot.y],
                    goal=[self.match.ball.x, self.match.ball.y],
                    rand_area=[0, field_size[0]],
                    obstacle_list=obstacle_list,
                    play_area=[0, field_size[0], 0, field_size[1]],
                    path_resolution=0.005,
                    expand_dis=0.025,
                    robot_radius=self.robot.dimensions['L'] * 1.5/2,
                    max_iter=1000,
                )

                rrt_path = self.rrt_.planning(False)
                if rrt_path:
                    self.path = rrt_path[::-1]
                    self.iteration += 1
            
            self.last_calculation += 1
            time.sleep(1/30)
            
            



        


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
        
        point = self.path[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.1:
            print("Next Point!")
            print(len(self.path))
            self.path.rotate(-1)

        
        return self.path[0]



    def decide(self):
        api.Api().send_custom_data(
            {
                'circuit': list(self.planning.path),
                'robot': {'x': self.robot.x, 'y': self.robot.y},
                'obstacles': [{'x': r.x, 'y': r.y} for r in self.match.opposites]
            }
        )
        point = self.next_point()
        return point
