
import math
from commons.math import point_in_rect
from entities.plays.playbook import Play, Playbook, Trigger

class PlayerPlaybook(Playbook):
    def __init__(self, coach, robot):
        super().__init__(coach)
        self.robot = robot

    def update(self):
        self._transition_if_have()
        # if self.robot.robot_id == 8:
        #      print(self.robot.robot_id, self.robot.strategy ,self.actual_play)
        return self.plays[self.actual_play].update()


class PlayerPlay(Play):
    def __init__(self, match, robot):
        super().__init__(None)
        self.match = match
        self.robot = robot

    def update(self):
        pass
    
    def add_transition(self, trigger, destination):
        self.transitions.append( (trigger, destination) )


class OnDefensiveTransitionTrigger(Trigger):
    def __init__(self, robot, match, in_defense=True, defensive_distance=0.25):
        super().__init__()
        self.match = match
        self.robot = robot

        self.in_defense = in_defense
        self.defensive_distance = defensive_distance

        self.goal_pos = [
            self.match.game.field.get_dimensions()[0],
            self.match.game.field.get_dimensions()[1]/2
        ]

    def evaluate(self, *args, **kwargs):

        if self.in_defense:
            return self.match.ball.x < self.defensive_distance
        else:
            return self.match.ball.x > self.defensive_distance

class OnInsideBox(Trigger):
    def __init__(self, match, box, outside=False):
        super().__init__()
        self.ball = match.ball
        self.box = box
        self.outside = outside

    def evaluate(self, *args, **kwargs):
        if self.outside:
            return not point_in_rect([self.ball.x, self.ball.y], self.box)
        return point_in_rect([self.ball.x, self.ball.y], self.box)

class OnCorners(Trigger):
    def __init__(self, match, corners, outside=False):
        super().__init__()
        self.ball = match.ball
        self.corners = corners
        self.outside = outside

    def evaluate(self, *args, **kwargs):
        eval = (self.ball.y < self.corners[0] or self.ball.y > self.corners[1])
        if self.outside:
            return not eval
        return eval

class OnStuckTrigger(Trigger):
    def __init__(self, robot, seconds_stuck=1):
        super().__init__()
        self.robot = robot
        self.seconds_stuck = seconds_stuck

    def evaluate(self, *args, **kwargs):
        print('aaaaaa')
        return self.robot.is_stuck()

class OnAttackerPushTrigger(Trigger):
    def __init__(self, robot, match, min_angle=0.60):
        super().__init__()
        self.match = match
        self.robot = robot
        self.min_angle = min_angle

        self.goal_pos = [
            self.match.game.field.get_dimensions()[0],
            self.match.game.field.get_dimensions()[1]/2
        ]

        
    def evaluate(self, *args, **kwargs):

        min_angle_ball_to_goal = -math.atan2(
            (self.match.ball.y - self.goal_pos[1]), 
            (self.match.ball.x - self.goal_pos[0])
        )
        angle_robot_to_ball = -math.atan2(
            (self.robot.y - self.match.ball.y), 
            (self.robot.x - self.match.ball.x )
        )
        
        min_angle_to_goal = abs(min_angle_ball_to_goal - angle_robot_to_ball)

        return min_angle_to_goal <= self.min_angle

class AndTransition(Trigger):
    def __init__(self, triggers_list):
        super().__init__()
        self.triggers_list = triggers_list

    def evaluate(self, *args, **kwargs):
        eval = [t.evaluate(*args, **kwargs) for t in self.triggers_list]
        return all(eval)
    
class OrTransition(Trigger):
    def __init__(self, triggers_list):
        super().__init__()
        self.triggers_list = triggers_list

    def evaluate(self, *args, **kwargs):
        eval = [t.evaluate(*args, **kwargs) for t in self.triggers_list]
        return any(eval)

class NotTransition(Trigger):
    def __init__(self, transition):
        super().__init__()
        self.transition = transition

    def evaluate(self, *args, **kwargs):
        return not self.transition.evaluate()
    
class CheckAngle(Trigger):
    def __init__(self, robot, ball):
        super().__init__()
        self.robot = robot
        self.ball = ball

    def evaluate(self, *args, **kwargs):
        if not ((self.robot.theta < 5 and self.robot.theta > 4.6) or (self.robot.theta < 1.7 and self.robot.theta > 1.35)):
            if self.ball.x > .6:
                return True
        return False

class OnNextTo(Trigger):
    def next_to(self, p1, p2, err=0.05, far=False):
        res = None

        if callable(p2):
            p2 = list(p2(self.p1.strategy))

        if type(p1) == list:
            p1x, p1y = p1[0], p1[1]
        else:
            p1x, p1y = p1.x, p1.y
        if type(p2) == list:
            p2x, p2y = p2[0], p2[1]
        else:
            p2x, p2y = p2.x, p2.y

        dx = p1x - p2x
        dy = p1y - p2y

        if math.sqrt(dx**2 + dy**2) < err:
            res = True
        else:
            res = False
        
        if far:
            return not res
        return res

    def __init__(self, p1, p2, distance, far=False):
        super().__init__()
        self.p1 = p1
        self.p2 = p2
        self.distance = distance

        self.far = far

    def evaluate(self, *args, **kwargs):
        is_nexto = self.next_to(self.p1, self.p2, self.distance, self.far)
        return is_nexto

class RobotOnInsideBox(Trigger):
    def __init__(self, match, box, robot, outside=False):
        super().__init__()
        self.box = box
        self.robot = robot
        self.outside = outside

    def evaluate(self, *args, **kwargs):
        if self.outside:
            return not point_in_rect([self.robot.x, self.robot.y], self.box)
        return point_in_rect([self.robot.x, self.robot.y], self.box)

# class GoalkeeperPush(Trigger):
#     def __init__(self, match):
#         super.__init__()
#         self.match = match
#         self.goalkeeper = None
#         for r in self.match.robots:
#             if r.strategy.name == "Goalkeeper":
#                 self.goalkeeper = r
    
#     def evaluate(self):
#         return isinstance(self.goalkeeper.strategy.get_play(),PushPlay)
