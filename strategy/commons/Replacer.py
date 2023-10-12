from controller.PID_control import PID_control, PID_W_control
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import PlayerPlaybook, PlayerPlay, OnNextTo
from algorithms.limit_cycle import LimitCycle


class Target:
    def __init__(self, target_dict):
        self.x = 0
        self.y = 0
        self.target_dict = target_dict
        self.guide_x = 0
        self.guide_y = 0

    def update(self, foul, quadrant, mine):
        if foul == 'KICKOFF':
            target = self.target_dict[foul][mine]['robot']
            guide = self.target_dict[foul][mine]['guide']
        elif foul == 'FREE_BALL':
            target = self.target_dict[foul][quadrant]['robot']
            guide = self.target_dict[foul][quadrant]['guide']
        elif foul == 'PENALTY_KICK':
            target = self.target_dict[foul][mine]['robot']
            guide = self.target_dict[foul][mine]['guide']
        elif foul == 'GOAL_KICK':
            target = self.target_dict[foul][mine]['robot']
            guide = self.target_dict[foul][mine]['guide']
        else:
            target = [0, 0]
            guide = [0, 0]

        self.x = target[0]
        self.y = target[1]

        self.guide_x = guide[0]
        self.guide_y = guide[1]


class Position(PlayerPlay):
    def __init__(self, match, robot, target):
        super().__init__(match, robot)
        self.target = target

    def get_name(self):
        return f"<{self.robot.get_name()} Position Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs={'V_MIN': 0, 'V_MAX':0.1 ,  'K_RHO': 6}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.target.x, self.target.y

    def start(self):
        pass


class Far(PlayerPlay):
    def __init__(self, match, robot, target):
        super().__init__(match, robot)
        self.target = target

    def get_name(self):
        return f"<{self.robot.get_name()} Far Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_W_control
        controller_kwargs={'V_MIN': 0, 'V_MAX':0.1}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
        self.limit_cycle = LimitCycle(self.match)

    def update(self):
        self.limit_cycle.set_target((self.target.x, self.target.y))
        self.limit_cycle.del_all_obstacles()

        for robot in self.match.robots:
            self.limit_cycle.add_obstacle((robot.x, robot.y, 0.11, 0))

        for robot in self.match.opposites:
            self.limit_cycle.add_obstacle((robot.x, robot.y, 0.13, 0))

        self.limit_cycle.add_obstacle((self.match.ball.x, self.match.ball.y, 0.13, 0))

        return self.limit_cycle.compute(self.robot)

    def start(self):
        pass


class Angle(PlayerPlay):
    def __init__(self, match, robot, target):
        super().__init__(match, robot)
        self.target = target

    def get_name(self):
        return f"<{self.robot.get_name()} Angle Planning>"

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {'V_MIN': 0, 'V_MAX': 0, 'TWO_SIDES': False, 'K_P': -5}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def update(self):
        return self.target.guide_x, self.target.guide_y

    def start(self):
        pass


class Replacer(Strategy):
    def __init__(self, match,target_dict, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control)
        self.target = Target(target_dict)
        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        # Criando Path Planning baseado em Astar
        pos = Position(self.match, self.robot, self.target)
        pos.start()
        angle = Angle(self.match, self.robot, self.target)
        angle.start()
        far = Far(self.match, self.robot, self.target)

        self.playerbook.add_play(pos)
        self.playerbook.add_play(angle)
        self.playerbook.add_play(far)

        on_point = OnNextTo(self.target, self.robot, 0.02)
        off_point = OnNextTo(self.target, self.robot, 0.02, True)
        on_near = OnNextTo(self.target, self.robot, 0.05)

        pos.add_transition(on_point, angle)
        angle.add_transition(off_point, pos)
        far.add_transition(on_near, pos)

        # Estado inicial é o astar
        self.playerbook.set_play(far)

    def decide(self):
        self.target.update(self.match.match_event['event'],
                           self.match.match_event['quadrant'],
                           str(self.match.match_event['mine']))
        res = self.playerbook.update()
        return res
