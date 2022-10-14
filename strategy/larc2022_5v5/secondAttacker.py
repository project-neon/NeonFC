from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.potential_fields import fields
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR
from strategy.BaseStrategy import Strategy
import algorithms
import math
from strategy.larc2022_5v5.commons import AstarPlanning, aim_projection_ball

from strategy.utils.player_playbook import AndTransition, OnInsideBox, OnNextTo, PlayerPlay, PlayerPlaybook

class DefendPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(
            match, 
            robot
        )

    def start_up(self):
        super().start_up()
        controller = PID_control
        controller_kwargs = {'max_speed': 2, 'max_angular': 3600, 'kp': 140}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        self.defend = algorithms.fields.PotentialField(
            self.match,
            name="{}|DefendBehaviour".format(self.__class__)
        )

        self.defend.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=lambda m, r=self.robot: (
                    m.ball.x + (math.cos(math.pi/3) if (m.ball.y < r.y) else math.cos(5*math.pi/3)) * 0.1,
                    m.ball.y + (math.sin(math.pi/3) if (m.ball.y < r.y) else math.sin(5*math.pi/3)) * 0.1
                ),                                                                                                                                                                                                                                                                                                                                          
                radius = 0.1,
                radius_max = 2,
                clockwise = lambda m, r=self.robot: (m.ball.y < r.y),
                decay=lambda x: 1,
                multiplier = 1
            )
        )
        self.defend.add_field(
            algorithms.fields.LineField(
                self.match,
                target= [0, self.match.game.field.get_dimensions()[1]/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi/2,
                line_size = (self.match.game.field.get_small_area("defensive")[3]/2),
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: 1,
                multiplier = -2
            )
        )
        self.defend.add_field(
            algorithms.fields.LineField(
                self.match,
                target= [0, self.match.game.field.get_dimensions()[1]/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi,
                line_size = 0.22,
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: 1,
                multiplier = -2
            )
        )

        for robot in self.match.robots:
            if robot.get_name() == self.robot.get_name():
                continue
            self.defend.add_field(
                fields.PointField(
                    self.match,
                    target = lambda m, r=robot: (
                        r.x,
                        r.y
                    ),
                    radius = .3,
                    radius_max = .3,
                    decay = lambda x: -1,
                    multiplier = 1
                )
            )

    def get_name(self):
        return f"<{self.robot.get_name()} Defend Potential Field Planning>"

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        dt = 0.05
        res = self.defend.compute(robot_pos)
        res[0] = self.robot.x + res[0] * dt
        res[1] = self.robot.y + res[1] * dt
        return res

class RightAttackerPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.robot_w = self.robot_h = 0.075

        self.g_hgr = (self.field_h/2)+0.185
        self.g_lwr = (self.field_h/2)-0.185

    def start_up(self):
        super().start_up()
        controller = TwoSidesLQR
        controller_kwargs = {}
        self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def get_name(self):
        return f"<{self.robot.get_name()} Right Attack Potential Field Planning>"

    def start(self):
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
        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue
            self.push.add_field(
                fields.PointField(
                    self.match,
                    target = lambda m, r=robot: (
                        r.x,
                        r.y
                    ),
                    radius = .3,
                    radius_max = .3,
                    decay = lambda x: -1,
                    multiplier = 1
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

    def update(self):
        p = (self.match.ball.y - self.robot.y)/(self.match.ball.x - self.robot.x)
        ball = self.match.ball

        if ball.x > self.field_w-0.35 and self.field_h/2-0.4 < ball.y < self.field_h/2+0.4:
            x = self.field_w
            left_proj = p*(x - self.robot.x) + self.robot.y + (self.robot_w/2)
            right_proj = p*(x - self.robot.x) + self.robot.y - (self.robot_w/2)

            if left_proj < self.g_hgr and right_proj > self.g_lwr:
                behaviour = self.push
            else:
                return self.use_astar([ball.x - 0.3, ball.y])

        else:
            if ball.x > self.field_w/4:
                if self.field_h/2-0.4 < ball.y < self.field_h/2+0.4:
                    return self.use_astar([ball.x - 0.3, ball.y])
                return self.use_astar([ball.x - 0.3, (self.field_h/2)])
            else:
                return self.use_astar(self.defend_position(self.match))

        return behaviour.compute([self.robot.x, self.robot.y])

class SecondAttacker(Strategy):
    def __init__(self, match, name="RightAttacker"):
        controller_kwargs = {'max_speed': 2, 'max_angular': 6000, 'kp': 180}
        super().__init__(match, name, controller=PID_control, controller_kwargs=controller_kwargs)

        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        field_dim = self.match.game.field.get_dimensions()

        astar = AstarPlanning(self.match, self.robot)
        astar.start()

        defend_potentialfield = DefendPlanning(self.match, self.robot)
        defend_potentialfield.start()

        rightattack_potentialfield = RightAttackerPlanning(self.match, self.robot)
        rightattack_potentialfield.start()

        self.playerbook.add_play(astar)
        self.playerbook.add_play(defend_potentialfield)
        self.playerbook.add_play(rightattack_potentialfield)

        # Transicao para caso esteja perto da bola ( < 10 cm)
        next_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.60)
        # Transicao para caso esteja longe da bola ( > 20 cm)
        far_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.80, True)

        on_defensive_sector_transition = OnInsideBox(self.match, [0, 0, field_dim[0]/2, field_dim[1]/2])
        on_offensive_sector_transition = OnInsideBox(self.match, [field_dim[0]/2, 0, field_dim[0]/2, field_dim[1]/2])

        astar.add_transition(
            AndTransition(
                [on_defensive_sector_transition, next_to_ball_transition]
            ), 
            defend_potentialfield
        )

        astar.add_transition(
            AndTransition(
                [on_offensive_sector_transition, next_to_ball_transition]
            ), 
            rightattack_potentialfield
        )

        defend_potentialfield.add_transition(far_to_ball_transition, astar)
        defend_potentialfield.add_transition(on_offensive_sector_transition, astar)

        rightattack_potentialfield.add_transition(far_to_ball_transition, astar)

        self.playerbook.set_play(astar)
    

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        res = self.playerbook.update()
        return res
