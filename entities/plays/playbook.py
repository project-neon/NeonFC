import math
import time
import strategy

class Playbook(object):
    def __init__(self, coach):
        self.coach = coach
        self.plays = {}
        self.actual_play = None
    
    def add_play(self, play):
        self.plays[play.get_name()] = play

    def set_play(self, play):
        if play.get_name() in self.plays.keys():
            self.actual_play = play.get_name()
            self.plays[self.actual_play].start_up()
        else:
            raise KeyError("Play is not defined on actual Playbook")
    
    def get_actual_play(self):
        return self.plays[self.actual_play]

    def _transition_if_have(self):
        for transition, next_play in self.plays[self.actual_play].transitions:
            if transition.evaluate(self.coach, self.plays[self.actual_play]):
                self.set_play(next_play)
                return

    def update(self):
        self._transition_if_have()
        self.plays[self.actual_play].update()

class Trigger(object):
    def __init__(self):
        pass

    def evaluate(self, coach, actual_play):
        return False

class Play(object):
    def __init__(self, coach):
        self.coach = coach
        self.transitions = []
        self.start_running_time = 0

    def start_up(self):
        self.start_running_time = time.time()
    
    def get_running_time(self):
        return time.time() - self.start_running_time

    def get_positions(self, foul, team_color, foul_color, quadrant):
        return None

    def get_name(self):
        return self.__class__

    def tear_down(self):
        pass

    def update(self):
        pass
    
    def add_transition(self, trigger, destination):
        self.transitions.append( (trigger, destination) )

class OneOneOnePlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.constraints = [
            #estratégia - função eleitora - robot_id
            (strategy.larc2020.GoalKeeper(self.match), self._elect_goalkeeper, 0),
            (strategy.larc2020.Attacker(self.match), self._elect_attacker, 0),
            (strategy.larc2020.MidFielder(self.match), self._elect_midfielder, 0)
        ]

    def update(self):
        super().update()

        robots = [r.robot_id for r in self.match.robots]

        for strategy, fit_fuction, priority in self.constraints:
            elected = -1
            best_fit = -99999
            for robot_id in robots:
                robot_fit = fit_fuction(self.match.robots[robot_id])
                if (robot_fit > best_fit):
                    best_fit = robot_fit
                    elected = robot_id
            
            priority = elected
            if self.match.robots[elected].strategy is None:
                self.match.robots[elected].strategy = strategy
            elif self.match.robots[elected].strategy.name != strategy.name:
                self.match.robots[elected].strategy = strategy
                self.match.robots[elected].start()
            robots.remove(elected)

    def _elect_attacker(self, robot):

        is_behind = 2 if robot.x > self.match.ball.x else 1

        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_ball * is_behind

    def _elect_goalkeeper(self, robot):
        dist_to_goal = math.sqrt(
            (robot.x - 0)**2 + (robot.y - 0.65)**2
        )
        return 1000 - dist_to_goal

    def _elect_midfielder(self, robot):
        return 1

class UnstuckPlay(Play):
    def __init__(self, coach):
        super().__init__(coach)
        self.match = self.coach.match
        self.constraints = [
            #estratégia - função eleitora - robot_id
            (strategy.larc2020.GoalKeeper(self.match), self._elect_goalkeeper, 0),
            (strategy.larc2020.Attacker(self.match), self._elect_attacker, 0),
            (strategy.iron2021.Avoid(self.match), self._elect_midfielder, 0)
        ]

    def update(self):
        super().update()

        robots = [r.robot_id for r in self.match.robots]

        for strategy, fit_fuction, priority in self.constraints:
            elected = -1
            best_fit = -99999
            for robot_id in robots:
                robot_fit = fit_fuction(self.match.robots[robot_id])
                if (robot_fit > best_fit):
                    best_fit = robot_fit
                    elected = robot_id
            
            priority = elected
            if self.match.robots[elected].strategy is None:
                self.match.robots[elected].strategy = strategy
            elif self.match.robots[elected].strategy.name != strategy.name:
                self.match.robots[elected].strategy = strategy
                self.match.robots[elected].start()
            robots.remove(elected)

    def _elect_attacker(self, robot):

        is_behind = 2 if robot.x > self.match.ball.x else 1

        dist_to_ball = math.sqrt(
            (robot.x - self.match.ball.x)**2 + (robot.y - self.match.ball.y)**2
        )
        return 1000 - dist_to_ball * is_behind

    def _elect_goalkeeper(self, robot):
        dist_to_goal = math.sqrt(
            (robot.x - 0)**2 + (robot.y - 0.65)**2
        )
        return 1000 - dist_to_goal

    def _elect_midfielder(self, robot):
        return 1

class WaitFor(Trigger):
    def __init__(self, timeout):
        """
        stuck_timeout: time to active transition in seconds
        """
        super().__init__()
        self.timeout = timeout
    
    def evaluate(self, coach, actual_play):
        return (self.timeout - actual_play.get_running_time()) <= 0

class StuckRobots(Trigger):
    def __init__(self, stuck_strategies=[]):
        """
        stuck_timeout: time to active transition in seconds
        """
        super().__init__()
        self.stuck_strategies = stuck_strategies
    
    def evaluate(self, coach, actual_play):
        match = coach.match

        robots = [r for r in match.robots]
        dict_robots = {
            r.strategy.name: r for r in robots if r.strategy
        }

        if all([ dict_robots[st].is_stuck() for st in self.stuck_strategies if dict_robots.get(st)]):
            return True
        
        return False

class OnFreeBall(Trigger):
    def __init__(self, referee):
        super().__init__()
        self.referee = referee

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "FREE_BALL"

class OnKickOff(Trigger):
    def __init__(self, referee):
        super().__init__()
        self.referee = referee

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "KICK_OFF"

class OnFreeKick(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "FREE_KICK" and self.team_color.upper() == self.referee.get_color()

class OnPenaltyKick(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        # print(foul, self.referee.get_color())
        return foul == "PENALTY_KICK" and self.team_color.upper() == self.referee.get_color()

class OnGoalKick(Trigger):
    def __init__(self, referee, team_color):
        super().__init__()
        self.referee = referee
        self.team_color = team_color

    def evaluate(self, coach, actual_play):
        foul = self.referee.get_foul()
        return foul == "GOAL_KICK" and self.team_color.upper() == self.referee.get_color()
