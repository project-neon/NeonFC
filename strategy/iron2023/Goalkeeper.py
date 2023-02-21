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
            'V_MAX': 100,
            'V_MIN': 100
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
        ball = self.match.ball
        side = ball.y - self.robot.y
        x = self.robot.x + 0.5*math.cos(self.robot.theta + side*math.pi/2)
        y = self.robot.y + 0.5*math.sin(self.robot.theta + side*math.pi/2)
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
            'V_MAX': 100,
            'V_MIN': 100
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
            'K_RHO': 600,
            'V_MAX': 150,
            'V_MIN': 0
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

class FollowBallPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2

    def get_name(self):
        return f"<{self.robot.get_name()} Follow Ball>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 1750,
            'V_MAX': 150,
            'V_MIN': 0,
            'W_MAX': 0
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        ball = self.match.ball

        projection_limit = 0.05365

        projection_point = ball.y + (5/60)*ball.vy

        bounded_projection = min( max( projection_point, ball.y - projection_limit), ball.y + projection_limit )

        y = min( max(bounded_projection, self.goal_right), self.goal_left )
        return .075, y

class ForwardCornerPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2

        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2

    def get_name(self):
        return f"<{self.robot.get_name()} Forward Corners Play>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 600,
            'V_MAX': 150,
            'V_MIN': 0
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        ball = self.match.ball

        normalized_x = lambda x: (.75-x)/(.75-.15)
        left_normalized_y = lambda y: (1.3-y)/(1.3-.85)
        right_normalized_y = lambda y: (.45-y)/(.45-0)

        relative_y_left = self.field_h/2 +.1 + .1*(.85*normalized_x(ball.x)+.15*left_normalized_y(ball.y))
        relative_y_right = self.field_h/2 -.1 - .1*(.85*normalized_x(ball.x)+.15*right_normalized_y(ball.y))

        if ball.y > self.goal_area_left:
            return .075, relative_y_left
        elif ball.y < self.goal_area_right:
            return .075, relative_y_right

class ForwardMiddlePlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2

        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2

    def get_name(self):
        return f"<{self.robot.get_name()} Forward Middles Play>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'K_RHO': 600,
            'V_MAX': 150,
            'V_MIN': 0
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        ball = self.match.ball
        side = np.sign(ball.y - self.field_h/2)
        func_y = lambda x: math.exp(-7.5*(x+.065)) + .65

        return .075, side*func_y(ball.y)

class StationaryPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Stationary>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 0,
            'V_MIN': 0,
            'W_MAX': 0
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        return self.robot.x, self.robot.y

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
        defend_corner.start()

        follow_ball = FollowBallPlay(self.match, self.robot)
        follow_ball.start()

        forward_corner = ForwardCornerPlay(self.match, self.robot)
        forward_corner.start()

        forward_middle = ForwardMiddlePlay(self.match, self.robot)
        forward_middle.start()

        self.playerbook.add_play(stationary)
        self.playerbook.add_play(spinning)
        self.playerbook.add_play(returning_to_goal)
        self.playerbook.add_play(aligning_angle)
        self.playerbook.add_play(pushing_ball)
        self.playerbook.add_play(defend_corner)
        self.playerbook.add_play(follow_ball)
        self.playerbook.add_play(forward_corner)
        # self.playerbook.add_play(forward_middle)

        outside_goal_area_transition = OutsideOfGoalAreaTrigger(self.match, self.robot)
        out_of_alignment_transition = OutOfAlignmentTrigger(self.match, self.robot)
        pushing_ball_transition = BallInGoalAreaCorner(self.match, self.robot)
        defend_corner_transition = BallInCornerTrigger(self.match, self.robot)
        follow_ball_transition = FollowBallTrigger(self.match, self.robot)
        forward_corner_transition = ForwardCornerTrigger(self.match, self.robot)
        # forward_middle_transition = ForwardMiddleTrigger(self.match, self.robot)

        stationary_transition = StationaryTrigger(self.match, self.robot, defend_corner_transition)

        # stationary.add_transition(defend_corner_transition, defend_corner)
        # defend_corner.add_transition(stationary_transition, stationary)

        returning_to_goal.add_transition(out_of_alignment_transition, aligning_angle)
        returning_to_goal.add_transition(pushing_ball_transition, pushing_ball)
        returning_to_goal.add_transition(defend_corner_transition, defend_corner)
        returning_to_goal.add_transition(follow_ball_transition, follow_ball)
        returning_to_goal.add_transition(forward_corner_transition, forward_corner)
        # returning_to_goal.add_transition(forward_middle_transition, forward_middle)

        aligning_angle.add_transition(outside_goal_area_transition, returning_to_goal)
        aligning_angle.add_transition(pushing_ball_transition, pushing_ball)
        aligning_angle.add_transition(defend_corner_transition, defend_corner)
        aligning_angle.add_transition(follow_ball_transition, follow_ball)
        aligning_angle.add_transition(forward_corner_transition, forward_corner)
        # aligning_angle.add_transition(forward_middle_transition, forward_middle)

        pushing_ball.add_transition(outside_goal_area_transition, returning_to_goal)
        pushing_ball.add_transition(defend_corner_transition, defend_corner)
        pushing_ball.add_transition(follow_ball_transition, follow_ball)
        pushing_ball.add_transition(forward_corner_transition, forward_corner)
        # pushing_ball.add_transition(forward_middle_transition, forward_middle)

        defend_corner.add_transition(outside_goal_area_transition, returning_to_goal)
        defend_corner.add_transition(out_of_alignment_transition, aligning_angle)
        defend_corner.add_transition(pushing_ball_transition, pushing_ball)
        defend_corner.add_transition(follow_ball_transition, follow_ball)
        defend_corner.add_transition(forward_corner_transition, forward_corner)
        # defend_corner.add_transition(forward_middle_transition, forward_middle)

        follow_ball.add_transition(outside_goal_area_transition, returning_to_goal)
        follow_ball.add_transition(out_of_alignment_transition, aligning_angle)
        follow_ball.add_transition(pushing_ball_transition, pushing_ball)
        follow_ball.add_transition(defend_corner_transition, defend_corner)
        follow_ball.add_transition(forward_corner_transition, forward_corner)
        # follow_ball.add_transition(forward_middle_transition, forward_middle)

        forward_corner.add_transition(outside_goal_area_transition, returning_to_goal)
        forward_corner.add_transition(out_of_alignment_transition, aligning_angle)
        forward_corner.add_transition(pushing_ball_transition, pushing_ball)
        forward_corner.add_transition(defend_corner_transition, defend_corner)
        forward_corner.add_transition(follow_ball_transition, follow_ball)
        # forward_corner.add_transition(forward_middle_transition, forward_middle)

        # forward_middle.add_transition(outside_goal_area_transition, returning_to_goal)
        # forward_middle.add_transition(out_of_alignment_transition, aligning_angle)
        # forward_middle.add_transition(pushing_ball_transition, pushing_ball)
        # forward_middle.add_transition(defend_corner_transition, defend_corner)
        # forward_middle.add_transition(follow_ball_transition, follow_ball)
        # forward_middle.add_transition(forward_corner_transition, forward_corner)

        self.playerbook.set_play(returning_to_goal)

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
        alignment_tolerance = .05
        if self.robot.x > self.goal_vertical_line - alignment_tolerance: return True
        elif self.robot.x < alignment_tolerance: return True
        elif self.robot.y > self.goal_area_left: return True
        elif self.robot.y < self.goal_area_right: return True
        else: return False

class OutOfAlignmentTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.outside = OutsideOfGoalAreaTrigger(match, robot)

    def evaluate(self, *args, **kwargs):
        theta = self.robot.theta
        tolerance = .1

        if self.outside.evaluate(): return False

        if math.pi/2 - tolerance < theta < math.pi/2 + tolerance:
            return False
        if -math.pi/2 - tolerance < theta < -math.pi/2 + tolerance:
            return False
        return True

class FollowBallTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.outside = OutsideOfGoalAreaTrigger(match, robot)
        self.align = OutOfAlignmentTrigger(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2

        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2

        self.goal_vertical_line = .15

    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if self.outside.evaluate() or self.align.evaluate():
            return False

        if self.goal_area_right < ball.y < self.goal_area_left and self.goal_vertical_line < ball.x < self.field_w/2:
            return True
        else: return False

class StationaryTrigger(Trigger):
    def __init__(self, match, robot, trigger):
        super().__init__()
        self.match = match
        self.robot = robot

        self.play = trigger
    
    def evaluate(self, *args, **kwargs):
        result = self.play.evaluate()
        return not result

class BallInGoalAreaCorner(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.outside = OutsideOfGoalAreaTrigger(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2
    
    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if self.outside.evaluate(): return False

        if 0 < ball.x < self.goal_vertical_line and (self.goal_left < ball.y < self.goal_area_left or self.goal_area_right < ball.y < self.goal_right):
            return True
        else: return False

class BallInCornerTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.outside = OutsideOfGoalAreaTrigger(match, robot)
        self.align = OutOfAlignmentTrigger(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2
    
    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if self.outside.evaluate() or self.align.evaluate():
            return False

        if 0 < ball.x < self.goal_vertical_line and (ball.y > self.goal_area_left or self.goal_area_right > ball.y):
            return True
        else: return False

class ForwardCornerTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.outside = OutsideOfGoalAreaTrigger(match, robot)
        self.align = OutOfAlignmentTrigger(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2
    
    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if self.outside.evaluate() or self.align.evaluate():
            return False

        if ball.x > self.goal_vertical_line and (ball.y > self.goal_area_left or ball.y < self.goal_area_right):
            return True
        else: return False

class TackleBallTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.outside = OutsideOfGoalAreaTrigger(match, robot)

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.goal_left = self.field_h/2 + .2
        self.goal_right = self.field_h/2 - .2

        self.goal_vertical_line = .15
        self.goal_area_left = self.field_h/2 + .7/2
        self.goal_area_right = self.field_h/2 - .7/2
    
    def evaluate(self, *args, **kwargs):
        ball = self.match.ball

        if self.outside.evaluate():
            return False

        ball_distance = math.sqrt( (ball.x - self.robot.x)**2 + (ball.y - self.robot.y)**2)

        if self.goal_right < ball.y < self.goal_left and ball.x < self.goal_vertical_line and ball_distance < 0.06:
            return True
        else: return False
