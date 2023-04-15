import math
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox
from entities.plays.playbook import Trigger
from controller import PID_control

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

        stationary = StationaryPlay(self.match, self.robot)
        stationary.start()

        returning_to_goal = Returning_to_Goal(self.match, self.robot)
        returning_to_goal.start()

        aligning_angle = Aligning_angle(self.match, self.robot)
        aligning_angle.start()

        # spinning = Spinning(self.match, self.robot)
        # spinning.start()

        pushing_ball = PushBall(self.match, self.robot)
        pushing_ball.start()

        defend_corner = BallInCorner(self.match, self.robot)
        defend_corner.start()

        follow_ball = FollowBallPlay(self.match, self.robot)
        follow_ball.start()

        forward_corner = ForwardCornerPlay(self.match, self.robot)
        forward_corner.start()

        # self.playerbook.add_play(spinning)
        self.playerbook.add_play(returning_to_goal)
        self.playerbook.add_play(aligning_angle)
        self.playerbook.add_play(pushing_ball)
        self.playerbook.add_play(defend_corner)
        self.playerbook.add_play(follow_ball)
        self.playerbook.add_play(forward_corner)
        self.playerbook.add_play(stationary)

        outside_goal_area = OnInsideBox(self.match, [], True)

        if self.playerbook.actual_play == None:
            self.playerbook.set_play()

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        return res


