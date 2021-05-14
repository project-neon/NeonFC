import math
import time
import api
import algorithms
import threading
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector

import json
import numpy as np



class AstarAttacker(Strategy):
    def __init__(self, match):
        super().__init__(match, 'AstarAttacker')

        self.astar = algorithms.AStar()

    def start(self, robot=None):
        super().start(robot=robot)
        self.astar.start()

    def reset(self, robot=None):
        super().reset()

        self.astar.reset()

        if robot:
            self.robot = robot

    def get_obstacles(self):
        foes = [ {'x': r.x, 'y': r.y, 'radius': 7.5} for r in self.match.opposites]
        friends = [ {'x': r.x, 'y': r.y, 'radius': 7.5} for r in self.match.robots if r.robot_id != self.robot.robot_id]

        return foes + friends


    def decide(self):
        a = time.time()
        print(threading.current_thread().name, 'run decide!')
        obstacles = self.get_obstacles()

        pos = [self.robot.x, self.robot.y]
        target = [self.match.ball.x, self.match.ball.y]

        self.astar.update_field(
            obstacles = obstacles
        )

        self.astar.update(
            pos,
            target
        )

        api.DataSender().get_node('Astar').capture(
            obstacles=obstacles,
            length= [len(self.astar.maze.matrix), len(self.astar.maze.matrix[0])],
            target=target,
            position=pos,
            maze=self.astar.maze.matrix,
            path=self.astar.path
        )

        node = self.astar.next_node(self.robot.x, self.robot.y)
        print('ends decide!', time.time() - a)
        return 0, 0

