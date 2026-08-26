from controller import NoController, PID_control
from entities.Robot import Robot
from match.match_real_life import MatchRealLife
from strategy.BaseStrategy import Strategy
from strategy.utils.player_playbook import OnNextTo, PlayerPlay, PlayerPlaybook


class Idle(PlayerPlay):
    def __init__(self, match: MatchRealLife, robot: Robot):
        super().__init__(match, robot)

    def get_name(self) -> str:
        return f"<{self.robot.get_name()} {self.__class__.__name__}>"

    def start_up(self) -> None:
        super().start_up()
        controller = NoController
        self.robot.strategy.controller = controller(self.robot)


class Follow(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self) -> str:
        return f"<{self.robot.get_name()} {self.__class__.__name__}>"

    def start_up(self) -> None:
        super().start_up()

        controller = PID_control
        self.robot.strategy.controller = controller(self.robot)

    def update(self):
        return self.match.ball.x, self.match.ball.y

# Game -> Match -> Coach -> Strategies -> PlayerPlaybook -> PlayerPlay -> Transitions
class Follower(Strategy):
    def __int__(self, match: MatchRealLife, name: str = "Dummy"):
        """
        Creates a concrete implementation of a Strategy.
        Represents one robot behaviour.

        Parameters:
            match (MatchRealLife): the real life match object
            name  (str): the name of the Strategy
            controller (object = controller.SimpleLQR): WIP
            controller_kwargs (dict = {}): WIP
        """
        super().__init__(self, match, name)
        self.playerbook: PlayerPlaybook | None = None

    def start(self, robot: Robot | None = None) -> None:
        """Attribute useful class components, such as:
        controller and PlayerBook's Plays and Transitions.

        Parameters:
        robot (Robot | None = None): the virtual representation for the physical robot,
        the one that will assume the current behaviour
        """
        # Uses the 'parent' class default behaviour,
        # which is basically attribute the default controller to the class
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        # @TODO: Add plays to player book
        follow = Follow(self.match, self.robot)
        self.playerbook.add_play(follow)

        idle = Idle(self.match, self.robot)
        self.playerbook.add_play(idle)

        # @TODO: Add transitions to the Play
        on_near_ball = OnNextTo(self.match.ball, self.robot, 0.1, False)
        off_near_ball = OnNextTo(self.match.ball, self.robot, 0.3, True)

        follow.add_transition(on_near_ball, idle)
        idle.add_transition(off_near_ball, follow)

        # @TODO: Add starting state for the Strategy
        self.playerbook.add_play(idle)

    def decide(self):
        res = self.playerbook.update()
        return res
