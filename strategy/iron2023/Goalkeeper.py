import math
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
            'V_MAX': 1.8,
            'V_MIN': 1.8
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

class StationaryPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Returning to Goal>"

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

        self.playerbook.add_play(stationary)
        self.playerbook.add_play(returning_to_goal)
        self.playerbook.add_play(aligning_angle)

        stationary_transition = StationaryTrigger()
        outside_goal_area_transition = OutsideOfGoalAreaTrigger(self.match, self.robot)
        out_of_alignment_transition = OutOfAlignmentTrigger(self.match, self.robot)

        stationary.add_transition(outside_goal_area_transition, returning_to_goal)
        returning_to_goal.add_transition(out_of_alignment_transition, aligning_angle)
        aligning_angle.add_transition(stationary_transition, stationary)

        self.playerbook.set_play(stationary)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        print(res)
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
        if self.robot.y > self.goal_area_left: return True
        if self.robot.y < self.goal_area_right: return True
        return False

class OutOfAlignmentTrigger(Trigger):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

    def evaluate(self, *args, **kwargs):
        theta = self.robot.theta
        tolerance = .4

        outside = OutsideOfGoalAreaTrigger(self.match, self.robot)
        print(outside.evaluate())
        if outside.evaluate(): return False

        if math.pi/2 - tolerance < theta < math.pi/2 + tolerance:
            return False
        if -math.pi/2 - tolerance < theta < -math.pi/2 + tolerance:
            return False
        return True

class StationaryTrigger(Trigger):
    def __init__(self):
        super().__init__()
    
    def evaluate(self, *args, **kwargs):
        return True
