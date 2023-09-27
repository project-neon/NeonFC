import math

from controller import PID_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from algorithms.trigonometry import trigonometry

RAD_X = 1
RAD_Y = 1
GOAL_POS = {"x": 0, "y": 0}


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
            ball.x, ball.y, GOAL_POS['x'], GOAL_POS['y'], RAD_X, RAD_Y)
        cX = self.robot.x
        cY = self.robot.y
        desiredCurrentAngle = math.atan2(cX - sX, cY - sY)
        print("x:{},y:{}\nbx:{},by:{}\nvecX:{},vecY:{},ang:{}\ndesired:{}".format(
            cX, cY, GOAL_POS['x'], GOAL_POS['y'], sX, sY, sAp, desiredCurrentAngle
        ))



class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_RSM2023", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)
        self.playerbook.add_play(StayInArea(self.match, self.robot))

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        print("Play: " + str(self.playerbook.actual_play))
        return res
