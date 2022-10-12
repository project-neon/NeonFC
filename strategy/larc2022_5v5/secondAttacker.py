from strategy.BaseStrategy import Strategy
from controller.simple_LQR import TwoSidesLQR
import algorithms
import math

class SecondAttacker(Strategy):
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

        self.avoid_area = algorithms.fields.PotentialField(
            self.match,
            name="{}|AvoidAreaBehaviour".format(self.__class__)
        )

        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|DefendBehaviour".format(self.__class__)
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

        self.defend.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m: (
                    m.ball.x + (math.cos(math.pi/3) if m.ball.y < 0.65 else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if m.ball.y < 0.65 else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.04,
                radius_max = 2,
                clockwise = lambda m: (m.ball.y < 0.65),
                decay=lambda x: 1,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 1
            )
        )

        # Avoid defensive area
        self.avoid_area.add_field(
            algorithms.fields.LineField(
                self.match,
                target= [self.match.game.field.get_dimensions()[0]+0.45 - self.match.game.field.get_dimensions()[0], 
                self.match.game.field.get_dimensions()[1]/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi/2,
                line_size = (self.match.game.field.get_small_area("defensive")[3]/2) + 0.09,
                line_dist = 0.23,
                line_dist_max = 0.23,
                decay = lambda x: 1,
                multiplier = -3
            )
        )

        self.push.add_field(self.avoid_area)
        self.defend.add_field(self.avoid_area)
    
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

        return r_v

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        p = (self.match.ball.y - self.robot.y)/(self.match.ball.x - self.robot.x)
        ball = self.match.ball

        attacker = [r for r in self.match.robots if r.robot_id != self.robot.robot_id]

        # if ball.x > self.field_w-0.35 and self.field_h/2-0.4 < ball.y < self.field_h/2+0.4:
        #     x = self.field_w
        #     left_proj = p*(x - self.robot.x) + self.robot.y + (self.robot_w/2)
        #     right_proj = p*(x - self.robot.x) + self.robot.y - (self.robot_w/2)

        #     if left_proj < self.g_hgr and right_proj > self.g_lwr:
        #         behaviour = self.push
        #     else:
        #         return self.use_astar([ball.x - 0.3, ball.y])

        # else:
        #     if ball.x > self.field_w/4:

        #         if self.field_h/2-0.4 < ball.y < self.field_h/2+0.4:
        #             return self.use_astar([ball.x - 0.3, ball.y])
        #         return self.use_astar([ball.x - 0.3, (self.field_h/2)])
        #     else:

        #         return self.use_astar(self.defend_position(self.match))

        if ball.x <= self.match.game.field.get_dimensions()[0]/2:
            if math.dist([self.robot.x, self.robot.y], [self.match.ball.x, self.match.ball.y]) > 0.1:
                return self.use_astar([self.match.ball.x, self.match.ball.y])
            else:
                behaviour = self.defend
        else:
            if math.dist([attacker[0].x, attacker[0].y], [ball.x, ball.y]) > 0.30:
                behaviour = self.push
            else:
                return self.use_astar([attacker[0].x-0.4, self.match.game.field.get_dimensions()[1]/2])

        
        if behaviour:
            return behaviour.compute([self.robot.x, self.robot.y])
        else:
            return self.use_astar([self.match.game.field.get_dimensions()[0]/2, self.match.game.field.get_dimensions()[1]/2])