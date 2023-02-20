from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.potential_fields import fields
from controller.PID_control import PID_control
from controller.simple_LQR import TwoSidesLQR
from strategy.BaseStrategy import Strategy
import algorithms
import math
from strategy.larc2022_5v5.commons import AstarPlanning, DefendPlanning, aim_projection_ball

from strategy.utils.player_playbook import AndTransition, OnInsideBox, OnNextTo, PlayerPlay, PlayerPlaybook
from entities.plays.playbook import IsAttackerSpin

def define_aim_point(match):
    field_h, field_w = match.game.field.get_dimensions()

    ball_speed = (match.ball.vx**2 + match.ball.vy**2)*.5
    ball_speed_vector = [match.ball.vx, match.ball.vy]
    goal_pos = [field_h, field_w/2]
    ball_pos = [match.ball.x, match.ball.y]
    ball_pos_proj = [match.ball.x * match.ball.vx, match.ball.y * match.ball.vy]

    projection = line_intersection(([field_h-0.2, field_w/2], [field_h+0.2, field_w/2]), (ball_pos, ball_pos_proj))

    if projection[0] > field_h-0.2 and projection[0] > field_h+0.2:
        aim_point = projection
    else:
        aim_point = goal_pos

    return aim_point
    

class Push(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.timeout = 10

        self.match = match
    def get_name(self):
        return f"<{self.robot.get_name()} Push Planning>"

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2, 'max_angular': 2000}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        pass
    def update(self):
        
        self.field_dim = self.match.game.field.get_dimensions()
        self.pb = [self.match.ball.x, self.match.ball.y] #ball position
        self.vb = [self.match.ball.vx, self.match.ball.vy] # ball speed vector
        if self.vb[0] == 0:
            self.vb[0] = 0.0000001
        self.vb_mod = (self.vb[0]**2 + self.vb[1]**2)**(1/2) # absolute ball speed
        self.pr = [self.robot.x, self.robot.y] # robot position
        self.vr = [self.robot.vx, self.robot.vy] # robot speed vector
        actual_play = self.robot.strategy.playerbook.get_actual_play()
        if self.pb[1] > 0.65:
            self.b = 0.55
        else:
            self.b = 0.75
        self.a = 0
        self.xc = 0
        self.delta = -0.06
        if (self.vb[1]/self.vb[0] - self.a) != 0:
            self.xc = (self.b - self.pb[1] - (self.vb[1]/self.vb[0])*self.pb[0])/((self.vb[1]/self.vb[0]) - self.a)
        else:
            self.xc = (self.b - self.pb[1] - (self.vb[1]/self.vb[0])*self.pb[0])/(0.00001)
        self.yc = -self.a*self.xc + self.b
        if self.timeout - actual_play.get_running_time() > 0:
            distance = ((self.yc - self.pb[1])**2 + (self.xc - self.pb[0])**2)**(1/2)
            if distance == 0:
                distance = 0.000001
            if ((self.pr[0]-self.pb[0])**2+(self.pr[1]-self.pb[1])**2)**(1/2) < 0.1:
                if self.robot.team_color == "blue":
                    if self.robot.y > self.field_dim[1]/2:
                        self.robot.strategy.spin = 500
                    else:
                        self.robot.strategy.spin = -500
                else:
                    if self.robot.y > self.field_dim[1]/2:
                        self.robot.strategy.spin = -500
                    else:
                        self.robot.strategy.spin = 500
            speed = min(2,7*(self.vb_mod/(distance**(1/4))))
            
            controller = PID_control
            controller_kwargs = {'max_speed':max(2,speed), 'max_angular': 3500}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)
          
            if self.pb[0] > 0.75 and self.vb_mod > 0.25:
                return -self.xc + self.delta, self.yc
            elif self.pb[0] > 0.75:
                return 1, self.b
            else:
                self.robot.strategy.playerbook.set_play(Mid(self.match, self.robot))
                return 0.75, 0.65

        else:
            self.robot.strategy.spin = 0
            self.robot.strategy.playerbook.set_play(Mid(self.match, self.robot))
        return 0,0

class Mid(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.match = match
        self.field_w, self.field_h = self.match.game.field.get_dimensions()
    def get_name(self):
        return f"<{self.robot.get_name()} Mid Planning>"

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2, 'max_angular': 2000}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        pass
    def update(self):
        if self.match.ball.y > self.field_h/2:
            return [self.field_w/2,self.field_h*(3/8)]
        if self.match.ball.y < self.field_h/2:
            return [self.field_w/2,self.field_h*(5/8)]
        
        return [self.field_w/2,self.field_h*(3/4)]
class Defend(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.match = match
        self.field_w, self.field_h = self.match.game.field.get_dimensions()
    def get_name(self):
        return f"<{self.robot.get_name()} defend Planning>"

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2, 'max_angular': 9000,"kp":60}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def start(self):
        pass
    def update(self):
        self.v = (self.match.ball.vx**2+self.match.ball.vy**2)**(1/2)
        controller_kwargs = {'max_speed': max(2,10*self.v), 'max_angular': 9000,"kp":60 + 100*self.v}
        return 0.3, min(0.85,max(0.45,self.match.ball.y + self.match.ball.vy/3))

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
        self.push.add_field(
            fields.LineField(
                self.match,
                target= [self.match.game.field.get_dimensions()[0] - self.match.game.field.get_dimensions()[0], 
                self.match.game.field.get_dimensions()[1]/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi/2,
                line_size = (self.match.game.field.get_small_area("defensive")[3]/2),
                line_dist = 0.2,
                line_dist_max = 0.2,
                decay = lambda x: 1,
                multiplier = -2
            )
        )
    
    def defend_position(self, match):
            if match.ball.y > self.g_hgr:
                return [self.field_w/2,self.field_h*(3/4)]
            if match.ball.y < self.g_lwr:
                return [self.field_w/2,self.field_h*(1/4)]
            
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

class Midfielder(Strategy):
    def __init__(self, match, name="Midfielder"):
        controller_kwargs = {'max_speed': 2, 'max_angular': 6000, 'kp': 180}
        super().__init__(match, name, controller=PID_control, controller_kwargs=controller_kwargs)
        self.spin = 0
        self.playerbook = None

    def start(self, robot=None):
        super().start(robot=robot)

        self.playerbook = PlayerPlaybook(self.match.coach, self.robot, True)

        field_dim = self.match.game.field.get_dimensions()

        astar = AstarPlanning(self.match, self.robot)
        astar.start()

        is_attacker_spin = IsAttackerSpin()
        

        defend_potentialfield = Defend(self.match, self.robot)
        defend_potentialfield.start()

        rightattack_potentialfield = RightAttackerPlanning(self.match, self.robot)
        rightattack_potentialfield.start()

        push_potential_field = Push(self.match, self.robot)
        push_potential_field.start()
        mid = Mid(self.match, self.robot)

        self.playerbook.add_play(astar)
        self.playerbook.add_play(defend_potentialfield)
        self.playerbook.add_play(rightattack_potentialfield)
        self.playerbook.add_play(push_potential_field)
        self.playerbook.add_play(mid)

        # Transicao para caso esteja perto da bola ( < 30 cm)
        next_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.30)
        # Transicao para caso esteja longe da bola ( > 40 cm)
        far_to_ball_transition = OnNextTo(self.robot, aim_projection_ball, 0.40, True)

        on_defensive_sector_transition = OnInsideBox(self.match, [0, 0, field_dim[0]/2, field_dim[1]])
        on_offensive_sector_transition = OnInsideBox(self.match, [3*field_dim[0]/4, 0.45, field_dim[0], 0.85])

        

        astar.add_transition(
            AndTransition(
                [on_defensive_sector_transition, next_to_ball_transition]
            ), 
            mid
        )

       # astar.add_transition(
       #     AndTransition(
       #         [on_offensive_sector_transition, next_to_ball_transition]
       #     ), 
       #     rightattack_potentialfield
       # )

        #defend_potentialfield.add_transition(far_to_ball_transition, astar)
        #defend_potentialfield.add_transition(on_offensive_sector_transition, astar)

        #rightattack_potentialfield.add_transition(far_to_ball_transition, astar)

        astar.add_transition(is_attacker_spin, push_potential_field)
        #defend_potentialfield.add_transition(is_attacker_spin, push_potential_field)
        rightattack_potentialfield.add_transition(is_attacker_spin, push_potential_field)
        mid.add_transition(is_attacker_spin, push_potential_field)
        mid.add_transition(on_offensive_sector_transition,push_potential_field)
        mid.add_transition(on_defensive_sector_transition,defend_potentialfield)
        defend_potentialfield.add_transition(on_offensive_sector_transition,mid)
        # Estado inicial é o astar
        self.playerbook.set_play(mid)
    

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):
        self.spin = 0

        res = self.playerbook.update()
        return res
    def update(self):
        if self.spin == 0:
            return self.controller.update()
        else:
            return -self.spin, self.spin
   