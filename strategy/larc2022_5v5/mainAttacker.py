import math
import threading

from collections import deque

from strategy.BaseStrategy import Strategy

from controller import PID_control

from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.limit_cycle import LimitCycle, Point

def aim_projection_ball(strategy):
    m = strategy.match
    b = strategy.match.ball

    ball = [b.x, b.y]
    goal_pos = [
        m.game.field.get_dimensions()[0],
        m.game.field.get_dimensions()[1]/2
    ]

    dir_to_goal_vector = [
        goal_pos[0] - ball[0], 
        goal_pos[1] - ball[1]
    ]
    angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

    return ball[0] - 0.2* math.cos(angle), ball[1] - 0.2 * math.sin(angle)

class AstarPlanning(threading.Thread):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.iteration = 0

        self.path = []

    def run(self):
        while True:
            if self.robot.strategy:
                self.path = voronoi_astar(
                    self.robot.strategy, self.match, aim_projection_ball
                )
                self.iteration += 1

                if self.path:
                    self.path = self.path[1:]

class LimitCyclePlanning():
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.limit_cycle = LimitCycle(
            self.match, 
            target_is_ball=True
        )

        self.desired_point = None

    def run(self):
        desired = 0, 0
        if self.robot.strategy:
            robot = Point(self.robot.x, self.robot.y)
            target = Point(self.match.ball.x, self.match.ball.y)

            if not (0 <= target.x <= 1.5 * 2) and not (0 <= target.y <= 1.3 * 2):
                target = Point(self.limit_cycle.target.x, self.limit_cycle.target.y)

            self.limit_cycle.update(robot, target, [])

            desired = self.limit_cycle.compute()


        return desired

class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control)

        self.planning = None
        self.path = []

        self.actual_iteration = -1

        self.is_on_attack = False



    def start(self, robot=None):
        super().start(robot=robot)

        self.astar = AstarPlanning(self.match, robot)
        self.limitcycle = LimitCyclePlanning(self.match, robot)

        self.astar.start()

        

    def next_point(self):
        if self.astar.iteration > self.actual_iteration:
            self.path = deque(self.astar.path)
            self.actual_iteration = self.astar.iteration
        
        if len(self.path) == 0:
            return self.robot.x, self.robot.y
        
        point = self.path[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.path.rotate(-1)

        
        return self.path[0]

    def next_to(self, p1, p2, err=0.05):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]

        if math.sqrt(dx**2 + dy**2) < err:
            return True

        return False

    def decide(self):
        
        robot_pos = [self.robot.x, self.robot.y]
        aim_pos = aim_projection_ball(self)
        
        if self.next_to(robot_pos, aim_pos, 0.40):
            return self.limitcycle.run()

        if len(self.astar.path):
            return self.next_point()
        
        return self.limitcycle.run()
