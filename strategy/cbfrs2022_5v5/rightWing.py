from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
from controller.simple_LQR import TwoSidesLQR
import algorithms

# class RightWing(DebugPotentialFieldStrategy):
class RightWing(Strategy):
    def __init__(self, match, name="RightWing"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.g_hgr = (self.field_h/2)+0.185
        self.g_lwr = (self.field_h/2)-0.185

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)
    
    def defend_position(self, match):
            if match.ball.y > self.g_hgr:
                return [match.ball.x - 0.05, self.field_h/2 - 0.15]
            else: 
                return [match.ball.x, (self.sa_y - 0.2)]

    def use_astar(self, target):
        target = target # target better not be the ball
        obstacles = [
            [r.x, r.y] for r in self.match.opposites] + [
            [r.x, r.y] for r in self.match.robots 
            if r.robot_id != self.robot.robot_id
        ]
        astar = algorithms.astar.PathAstar(self.match)
        robot_pos = [self.robot.x, self.robot.y]
        ball_pos = [self.match.ball.x, self.match.ball.y]
        if self.match.ball.x < self.robot.x:
            obstacles = obstacles + [ball_pos]
        r_v = astar.calculate(robot_pos, target, obstacles)
        # print(r_v)
        return r_v
    
    def decide(self):
        ball = self.match.ball

        if ball.y < self.g_lwr and ball.x > self.robot.x and ball.x > self.field_w/4:
            # behaviour = self.combat
            target_x = max(ball.x - 0.4, self.sa_w + 0.10)
            return self.use_astar([target_x, ball.y])
        else:
            if ball.x > self.sa_w + 0.4 and ball.y > self.sa_y - 0.15:
                # behaviour = self.attack
                target_x = max(ball.x - 0.6, self.sa_w + 0.10)
                return self.use_astar([target_x, (self.g_lwr - 0.1)])
            else:
                # behaviour = self.defend
                return self.use_astar(self.defend_position(self.match))
        
        # print(behaviour.name)

        # return behaviour.compute([self.robot.x, self.robot.y])