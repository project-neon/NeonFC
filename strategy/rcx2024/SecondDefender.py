import math
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlay, PlayerPlaybook, OnInsideBox, OnNextTo, AndTransition
from controller import PID_control, PID_W_control
from NeonPathPlanning import Point, LimitCycle


class MainPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Center Play>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        self.robot.strategy.controller = controller(self.robot)
        self.limit_cycle = LimitCycle()
        for r in self.match.robots:
            if r == self.robot:
                continue
            self.limit_cycle.add_obstacle(r, 0.075*1.4 + 0.1)


    def update(self):
        ball = self.match.ball
        main_st = next(filter(lambda r:r.strategy.name == "Main_Defender", self.match.robots))
        target = Point(ball.x-.15, main_st.y + .5*(.65-ball.y))
        self.limit_cycle.set_target(target=target)
        return self.limit_cycle.compute(self.robot)


class BlockCross(PlayerPlay):
    def __init__(self, match, robot):
        # super().__init__(match, "Main_Attacker", controller=PID_W_control)
        super().__init__(match, robot)
        self.dl = 0.000001

    def get_name(self):
        return f"<{self.robot.get_name()} Cross Blocker>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        self.robot.strategy.controller = controller(self.robot)
        self.limit_cycle = LimitCycle()
        for r in self.match.robots:
            if r == self.robot:
                continue
            self.limit_cycle.add_obstacle(r, 0.075*1.4 + 0.1)

    def update(self):
        left_target = Point(.3, .9)
        right_target = Point(.3, .4)

        if self.match.ball.y > .65:
            self.limit_cycle.set_target(target=left_target)
        else:
            self.limit_cycle.set_target(target=right_target)

        return self.limit_cycle.compute(self.robot)


class Wait(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Wait in area>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs={'V_MIN': 0, 'K_RHO': 1.5}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.position()

    def position(self):
        a = (.35, 1.1)
        b = (.35, 0.2)

        c = self.robot
        d = next(filter(lambda r:r.strategy.name != "Goalkeeper" and r!=self.robot, self.match.robots))

        # Calculate the distances between each robot and each fixed point
        distance_c_a = math.sqrt((c[0] - a[0]) ** 2 + (c[1] - a[1]) ** 2)
        distance_c_b = math.sqrt((c[0] - b[0]) ** 2 + (c[1] - b[1]) ** 2)
        distance_d_a = math.sqrt((d[0] - a[0]) ** 2 + (d[1] - a[1]) ** 2)
        distance_d_b = math.sqrt((d[0] - b[0]) ** 2 + (d[1] - b[1]) ** 2)

        # Assign the robots to the closer fixed point
        if distance_c_a + distance_d_b < distance_c_b + distance_d_a:
            return a
        else:
            return b


class LookAtBall(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Go to angle>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        controller_kwargs = {'V_MIN': 0, 'V_MAX': 0}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.match.ball.x, self.match.ball.y

class GoalPlay(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Goal play>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs={'V_MIN': 0, 'K_RHO': 1.5}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.position()

    def position(self):
        x = 0.3

        if self.match.ball.y >= 0.65:
            y = self.match.ball.y - 0.5
        else:
            y = self.match.ball.y + 0.5
        
        return (x, y)

class ShadowDefender(Strategy):
    def __init__(self, match):
        super().__init__(match, "Second_Defender", controller=PID_control)

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        main_play = MainPlay(self.match, self.robot)
        block_play = BlockCross(self.match, self.robot)
        wait_play = Wait(self.match, self.robot)
        angle_play = LookAtBall(self.match, self.robot)
        goal_play = GoalPlay(self.match, self.robot)

        self.playerbook.add_play(main_play)
        self.playerbook.add_play(wait_play)
        self.playerbook.add_play(angle_play)
        self.playerbook.add_play(block_play)
        self.playerbook.add_play(goal_play)

        on_cross_1 = OnInsideBox(self.match, [0, 0, .15, .25], False)
        on_cross_2 = OnInsideBox(self.match, [0, 1, .15, .25], False)
        off_cross_1 = OnInsideBox(self.match, [0, 0, .15, .3], True)
        off_cross_2 = OnInsideBox(self.match, [0, 1, .15, .3], True)
        on_area = OnInsideBox(self.match, [0, .25, .2, .8], False)
        off_area = OnInsideBox(self.match, [0, .25, .2, .8], True)
        on_position_1 = OnNextTo([.35, 1.1], self.robot, 0.1, False)
        off_position_1 = OnNextTo([.35, 1.1], self.robot, 0.1, True)
        on_position_2 = OnNextTo([.35, .2], self.robot, 0.1, False)
        off_position_2 = OnNextTo([.35, .2], self.robot, 0.1, True)
        # on_gk_push = GoalkeeperPush(self.match)
        on_angle_goal_1 = OnInsideBox(self.match, [-.5, -.1, .75, .3])
        off_angle_goal_1 = OnInsideBox(self.match, [-.5, -.1, .75, .3], False)
        on_angle_goal_2 = OnInsideBox(self.match, [-.5, 1.05, .75, .4])
        off_angle_goal_2 = OnInsideBox(self.match, [-.5, 1.05, .75, .4], False)

        main_play.add_transition(on_cross_1, block_play)
        main_play.add_transition(on_cross_2, block_play)
        block_play.add_transition(AndTransition([off_cross_1, off_cross_2]), main_play)

        main_play.add_transition(on_area, wait_play)
        wait_play.add_transition(off_area, main_play)
        wait_play.add_transition(on_position_1, goal_play)
        wait_play.add_transition(on_position_2, goal_play)
        angle_play.add_transition(AndTransition([off_position_1, off_position_2]), wait_play)
        angle_play.add_transition(off_area, main_play)

        main_play.add_transition(AndTransition([on_angle_goal_1, on_angle_goal_2]), goal_play)
        block_play.add_transition(AndTransition([on_angle_goal_1, on_angle_goal_2]), goal_play)
        wait_play.add_transition(AndTransition([on_angle_goal_1, on_angle_goal_2]), goal_play)
        angle_play.add_transition(AndTransition([on_angle_goal_1, on_angle_goal_2]), goal_play)

        goal_play.add_transition(AndTransition([off_angle_goal_1, off_angle_goal_2]), main_play)

        if self.playerbook.actual_play is None:
            self.playerbook.set_play(main_play)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        return res