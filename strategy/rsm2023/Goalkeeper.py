import math
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, AndTransition, OrTransition, NotTransition
from entities.plays.playbook import Trigger
from controller import PID_control
from commons.math import point_in_rect

class Returning_to_Goal(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Return to Goal>"

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
        return f"<{self.robot.get_name()} Align angle>"

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

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def get_name(self):
        return f"<{self.robot.get_name()} Spin>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {
            'V_MAX': 0,
            'V_MIN': 0,
            'W_MAX': 50000
        }
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return super().update()

    def start(self):
        pass

    def update(self):
        ball = self.match.ball
        side = np.sign(self.robot.y - ball.y)
        x = self.robot.x + 0.5*math.cos(self.robot.theta + side*math.pi/2)
        y = self.robot.y + 0.5*math.sin(self.robot.theta + side*math.pi/2)
        return x, y

class PushBall(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Push Ball>"

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
        return f"<{self.robot.get_name()} Defend Ball in Corner>"

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
        return self.robot.x, self.field_h/2 + side*0.2

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
            'V_MAX': 200,
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

        projection_rate = (ball.x-.15)/(1-.15)

        # projection_limit = 0.15*projection_rate

        projection_point = ball.y + projection_rate*ball.vy

        # bounded_projection = min( max( projection_point, ball.y - projection_limit), ball.y + projection_limit )

        y = min( max(projection_point, self.goal_right), self.goal_left )

        return self.robot.x, y

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

        side = np.sign(ball.y - self.field_h/2)

        y = self.field_h/2 + 0.2*side
        
        return self.robot.x, y

class StationaryPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Stationary>"

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
        return self.robot.x, .65

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "Goalkeeper_RSM2023", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        aligning_angle = Aligning_angle(self.match, self.robot)       # 1
        aligning_angle.start()

        returning_to_goal = Returning_to_Goal(self.match, self.robot) # 2
        returning_to_goal.start()

        follow_ball = FollowBallPlay(self.match, self.robot)          # 3
        follow_ball.start()

        defend_corner = BallInCorner(self.match, self.robot)          # 4
        defend_corner.start()

        forward_corner = ForwardCornerPlay(self.match, self.robot)    # 5
        forward_corner.start()

        pushing_ball = PushBall(self.match, self.robot)               # 6
        pushing_ball.start()

        stationary = StationaryPlay(self.match, self.robot)           # 7
        stationary.start()

        self.playerbook.add_play(returning_to_goal)
        self.playerbook.add_play(aligning_angle)
        self.playerbook.add_play(pushing_ball)
        self.playerbook.add_play(defend_corner)
        self.playerbook.add_play(follow_ball)
        self.playerbook.add_play(forward_corner)
        self.playerbook.add_play(stationary)

        not_aligned = OnAlignment(self.robot, .6*math.pi/2, True)    # 1
        aligned = OnAlignment(self.robot, .2*math.pi/2)              # !1
        outside_goal_area = OnReposition(self.robot)                 # 2
        on_follow_ball = OnInsideBox(self.match, [.15, .4, .6, .7])  # 3
        on_corner = OrTransition([                                   # 4
            OnInsideBox(self.match, [0, 1, .15, .3]),
            OnInsideBox(self.match, [0, 0, .15, .3])
        ])
        on_forward_corner = OrTransition([                           # 5
            OnInsideBox(self.match, [.15, 1, .6, .3]),
            OnInsideBox(self.match, [.15, 0, .6, .3])
        ])
        on_goal_area = OnInsideBox(self.match, [0, .4, .15, .7])     # 6

        on_attack_area = OnInsideBox(self.match, [.75, 0, .75, 1.3]) # 7

        aligning_angle.add_transition(outside_goal_area) # 1 -> 2
        aligning_angle.add_transition(AndTransition(     # 1 -> 3
            aligned, on_follow_ball
        ))
        aligning_angle.add_transition(AndTransition(     # 1 -> 4
            aligned, on_corner
        ))
        aligning_angle.add_transition(AndTransition(     # 1 -> 5
            aligned, on_forward_corner
        ))
        aligning_angle.add_transition(on_goal_area)      # 1 -> 6

        returning_to_goal.add_transition(AndTransition(  # 2 -> 1
            NotTransition(outside_goal_area), not_aligned
        ))

        follow_ball.add_transition(not_aligned)       # 3 -> 1
        follow_ball.add_transition(outside_goal_area) # 3 -> 2
        follow_ball.add_transition(on_corner)         # 3 -> 4
        follow_ball.add_transition(on_forward_corner) # 3 -> 5
        follow_ball.add_transition(on_goal_area)      # 3 -> 6
        follow_ball.add_transition(on_attack_area)    # 3 -> 7

        defend_corner.add_transition(not_aligned) # 4 -> 1
        defend_corner.add_transition(outside_goal_area) # 4 -> 2
        defend_corner.add_transition(follow_ball) # 4 -> 3
        defend_corner.add_transition(on_forward_corner) # 4 -> 5
        defend_corner.add_transition(on_goal_area) # 4 -> 6

        forward_corner.add_transition(not_aligned) # 5 -> 1
        forward_corner.add_transition(outside_goal_area) # 5 -> 2
        forward_corner.add_transition(follow_ball) # 5 -> 3
        forward_corner.add_transition(on_corner) # 5 -> 4
        forward_corner.add_transition(on_goal_area) # 5 -> 6
        forward_corner.add_transition(on_attack_area) # 5 -> 7

        pushing_ball.add_transition() # 6 -> x

        if self.playerbook.actual_play == None:
            self.playerbook.set_play(returning_to_goal)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        return res

class OnAlignment(Trigger):
    def __init__(self, robot, tolerance, outside=False):
        super().__init__()
        self.robot = robot
        self.outside = outside
        self.tolerance = tolerance

    def evaluate(self, *args, **kwargs):
        robot_theta = self.robot.theta

        if self.outside:
            return ( math.pi - self.tolerance < robot_theta < math.pi + self.tolerance or
                              -self.tolerance < robot_theta < self.tolerance)
        
        else:
            return ( math.pi/2 - self.tolerance < robot_theta <  math.pi/2 + self.tolerance or
                    -math.pi/2 - self.tolerance < robot_theta < -math.pi/2 + self.tolerance)

class OnReposition(Trigger):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.box = [0, .4, .15, .7]

    def evaluate(self, *args, **kwargs):
        return not point_in_rect([self.robot.x, self.robot.y], self.box)
