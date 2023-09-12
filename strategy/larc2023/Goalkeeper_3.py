import math

from controller import PID_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook


# Returns a vector present in a pseudo circle centered around origin with a radius of... radius
# The vector will always be the one with the shortest range between itself and the ballPos
def get_equivalent_circle_pos(ballPos, origin, radius):
    # ratio = inner_center / outer_center
    vec = {
        'x': ballPos['x'] - origin['x'],
        'y': ballPos['y'] - origin['y']
    }
    ang = math.atan2(vec['x'], vec['y'])
    vec = {
        'x': origin['x'] + math.sin(ang) * radius,
        'y': origin['y'] + math.cos(ang) * radius
    }
    return vec


class StayInArea(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Stay in area>"

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


class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_RSM2023", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        print(self.playerbook.actual_play)
        return res
