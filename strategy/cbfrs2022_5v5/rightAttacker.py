from dataclasses import field
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
from controller.simple_LQR import TwoSidesLQR
import algorithms
import math

# class RightAttacker(DebugPotentialFieldStrategy):
class RightAttacker(Strategy):
    def __init__(self, match, name="RightAttacker"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.robot_w = self.robot_h = 0.075

        self.g_hgr = (self.field_h/2)+0.185
        self.g_lwr = (self.field_h/2)-0.185

    def start(self, robot=None):
        super().start(robot=robot)

        self.push = algorithms.fields.PotentialField(
            self.match,
            name="{}|PushBehaviour".format(self.__class__)
        )

        self.push.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.1,
                multiplier = lambda m: max(1, (m.ball.vx**2 + m.ball.vy**2)**0.5 + 0.3),
                decay = lambda x : x
            )
        )
    
    def defend_position(self, match):
            if match.ball.y > self.g_hgr:
                return [self.sa_x + 0.25, self.field_h/2 + 0.25]
            if match.ball.y < self.g_lwr:
                return [self.sa_x + 0.25, self.field_h/2 - 0.25]
            
            return [self.sa_x + 0.4, self.field_h/2]

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

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        p = (self.match.ball.y - self.robot.y)/(self.match.ball.x - self.robot.x)
        ball = self.match.ball

        if ball.x > self.field_w-0.35 and self.field_h/2-0.4 < ball.y < self.field_h/2+0.4:
            x = self.field_w
            left_proj = p*(x - self.robot.x) + self.robot.y + (self.robot_w/2)
            right_proj = p*(x - self.robot.x) + self.robot.y - (self.robot_w/2)

            if left_proj < self.g_hgr and right_proj > self.g_lwr:
                behaviour = self.push
            else:
                # behaviour = self.follow
                return self.use_astar([ball.x - 0.3, ball.y])

        # elif ball.y <= self.sa_y + self.sa_h + 0.1 and ball.y >= self.sa_y - 0.1 and ball.x > self.sa_w + 0.3:
        #     behaviour = self.follow
        else:
            if ball.x > self.field_w/4:
                # behaviour = self.attack
                if self.field_h/2-0.4 < ball.y < self.field_h/2+0.4:
                    return self.use_astar([ball.x - 0.3, ball.y])
                return self.use_astar([ball.x - 0.3, (self.field_h/2)])
            else:
                # behaviour = self.defend
                return self.use_astar(self.defend_position(self.match))

        # print(ball.x > self.field_w-0.35 and self.field_h/2-0.4 < ball.y < self.field_h/2+0.4)
        # print(left_proj < self.g_hgr and right_proj > self.g_lwr)

        # print(behaviour.name)

        return behaviour.compute([self.robot.x, self.robot.y])