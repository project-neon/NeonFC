import math
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook
from entities.plays.playbook import Trigger
from controller import SimpleLQR, TwoSidesLQR, UniController, PID_control
from algorithms import UnivectorField
from algorithms import LimitCycle
from algorithms.potential_fields.fields import PotentialField, PointField, LineField, TangentialField

class Returning_to_Goal(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Returning to Goal>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 1,
            'V_MIN': 1
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        return (.075, 0.65)

class Aligning_angle(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Aligning angle>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 0,
            'V_MIN': 0
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        return self.robot.x, self.robot.y + .05

class Spinning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Spinning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 0,
            'V_MIN': 0,
            'W_MAX': 5000
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        x = self.robot.x + 0.5*math.cos(self.robot.theta + math.pi/2)
        y = self.robot.y + 0.5*math.sin(self.robot.theta + math.pi/2)
        return x, y

class PushBall(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Pushing Ball>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 1.8,
            'V_MIN': 1.8
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        ball = self.match.ball
        return ball.x, ball.y

class BallInCorner(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} Defending Ball in Corner>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 1.8,
            'V_MIN': 1.8
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        ball = self.match.ball
        side = np.sign(ball.y - self.robot.y)
        return .075, self.field_h/2 + side*0.2

class StationaryPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Stationary>"

    def start_up(self):
        super().start_up()
        controller = TwoSidesLQR
        controller_kwargs = {
            'l': 0.0135
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        return 0, 0

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_IRON2023", controller=TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        stationary = StationaryPlay(self.match, self.robot)
        stationary.start()

        returning_to_goal = Returning_to_Goal(self.match, self.robot)
        returning_to_goal.start()

        aligning_angle = Aligning_angle(self.match.coach, self.robot)
        aligning_angle.start()

        spinning = Spinning(self.match, self.robot)
        spinning.start()

        pushing_ball = PushBall(self.match, self.robot)
        pushing_ball.start()

        defend_corner = BallInCorner(self.match, self.robot)

        # self.playerbook.add_play(spinning)

        self.playerbook.add_play(stationary)
        # self.playerbook.add_play(returning_to_goal)
        # self.playerbook.add_play(aligning_angle)
        # self.playerbook.add_play(pushing_ball)
        self.playerbook.add_play(defend_corner)

        stationary_transition = StationaryTrigger(self.match, self.robot)
        outside_goal_area_transition = OutsideOfGoalAreaTrigger(self.match, self.robot)
        out_of_alignment_transition = OutOfAlignmentTrigger(self.match, self.robot)
        pushing_ball_transition = BallInGoalAreaCorner(self.match, self.robot)
        defend_corner_transition = BallInCornerTrigger(self.match, self.robot)

        # stationary.add_transition(outside_goal_area_transition, returning_to_goal)
        # stationary.add_transition(out_of_alignment_transition, aligning_angle)
        # returning_to_goal.add_transition(out_of_alignment_transition, aligning_angle)
        # returning_to_goal.add_transition(stationary_transition, stationary)
        # aligning_angle.add_transition(stationary_transition, stationary)
        # aligning_angle.add_transition(outside_goal_area_transition, returning_to_goal)
        # stationary.add_transition(pushing_ball_transition, pushing_ball)
        # pushing_ball.add_transition(stationary_transition, stationary)
        stationary.add_transition(defend_corner_transition, defend_corner)
        defend_corner.add_transition(stationary_transition, stationary)

        self.playerbook.set_play(stationary)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        print(self.playerbook.actual_play)
        return res

class OutsideOfGoalAreaTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2

    def evaluate(self, *args, **kwargs):
        if self.robot.x > self.goal_vertical_line: return True
        elif self.robot.y > self.goal_area_left: return True
        elif self.robot.y < self.goal_area_right: return True
        else: return False

class OutOfAlignmentTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

    def evaluate(self, *args, **kwargs):
        theta = self.robot.theta
        tolerance = .05

        outside = OutsideOfGoalAreaTrigger(self.match, self.robot)
        # print(outside.evaluate())
        if outside.evaluate(): return False

        if math.pi/2 - tolerance < theta < math.pi/2 + tolerance:
            return False
        if -math.pi/2 - tolerance < theta < -math.pi/2 + tolerance:
            return False
        return True

class StationaryTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot
    
    def evaluate(self, *args, **kwargs):
        ev = BallInGoalAreaCorner(self.match, self.robot)
        ev = ev.evaluate()
        return not (ev)

class BallInGoalAreaCorner(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2
    
    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if 0 < ball.x < self.goal_vertical_line and (self.goal_left < ball.y < self.goal_area_left or self.goal_area_right < ball.y < self.goal_right):
            return True
        else: return False

class BallInCornerTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2
    
    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if 0 < ball.x < self.goal_vertical_line and (ball.y > self.goal_area_left or self.goal_area_right > ball.y):
            return True
        else: False

'''
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

função que retorna o valor n limitado pelos maximo maxn e minimo minn
'''