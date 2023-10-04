import math

from controller import PID_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from algorithms.trigonometry import trigonometry
from strategy.utils.player_playbook import DefaultToTransition

RAD_X = .5
RAD_Y = .5
GOAL_POS = {"x": 0, "y": 0.65}


class StayInArea(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Patrol area>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1,
            'V_MIN': 0,
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        ball = self.match.ball
        sX, sY, sAp, sAn = trigonometry.get_closest_ellipse_position_pure(
            ball.x, ball.y + ball.vy, GOAL_POS['x'], GOAL_POS['y'], RAD_X, RAD_Y)
        print("Ball at {:.4f},{:.4f}, prediction vector located at {:.2f},{:.2f}".
              format(ball.x, ball.y, sX, sY))
        return sX, sY


class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_RSM2023", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)
        stay_in_area = StayInArea(self.match, self.robot)
        self.playerbook.add_play(stay_in_area)
        self.playerbook.set_play(stay_in_area)
        stay_in_area.add_transition(DefaultToTransition(), stay_in_area)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        return res
