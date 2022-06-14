from algorithms.astar import PathAstar
from strategy.BaseStrategy import Strategy

class AsScratch2(Strategy):
    def __init__(self, match):
        super().__init__(match, "AstarVoronoi")
        self.last_update = 0
    
    def start(self, robot=None):
        super().start(robot=robot)
        self.astar = PathAstar(self.match)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)
    
    def decide(self):
        robot_pos = [self.robot.x, self.robot.y]
        ball_pos = [self.match.ball.x, self.match.ball.y]

        obstacles = [
            [r.x, r.y] for r in self.match.opposites] + [
            [r.x, r.y] for r in self.match.robots 
            if r.robot_id != self.robot.robot_id
        ]

        r_v = self.astar.calculate(robot_pos, ball_pos, obstacles)

        return r_v
