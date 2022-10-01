
import math
from entities.plays.playbook import Play, Playbook, Trigger

class PlayerPlaybook(Playbook):
    def __init__(self, coach, robot):
        super().__init__(coach)
        self.robot = robot

    def update(self):
        self._transition_if_have()
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