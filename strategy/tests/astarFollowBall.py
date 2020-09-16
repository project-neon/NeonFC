import algorithims
from strategy.BaseStrategy import Strategy
from commons.math import unit_vector


class FollowBall(Strategy):
    def __init__(self, match):
        super().__init__(match)
        self.astar = algorithims.AStar()


    def start(self, robot=None, speed=1000, astar_timespan=0.1):
        super().start(robot=robot)

        self.speed = speed
        self.astar_timespan = astar_timespan

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.robot = robot

        self.astar.reset_calculation_timespan()


    def decide(self):
        self.astar.update_field(
            obstacles=[
                {
                    "x": r.x, 
                    "y": r.y
                } for r in self.match.opposites + self.match.robots if not (r.team_color == self.robot.team_color and r.robot_id == self.robot.robot_id)
            ]
        )

        self.astar.calculate_when(
            (self.robot.x, self.robot.y),
            (self.match.ball.x, self.match.ball.y),
            timespan = self.astar_timespan
        )

        objective = self.astar.next_node(self.robot.x, self.robot.y)

        desired = unit_vector([(objective[0] - self.robot.x), (objective[1] - self.robot.y)]) * self.speed
        return desired