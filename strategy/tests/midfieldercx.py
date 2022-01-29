import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from commons.math import point_in_rect

class MidFielderSupporter(Strategy):
    def __init__(self, match, name='MidFielderkkkkk', attacker='UFV-Attacker'):
        super().__init__(match, name, controller=controller.TwoSidesLQR)
        self.attacker = attacker
    
    def start(self, robot=None):
        super().start(robot=robot)

        VEL = 1

        self.sobra = algorithms.fields.PotentialField(self.match, name="SobraBehaviour")

        self.push = algorithms.fields.PotentialField(self.match, name="PushBehaviour")

        self.attack = algorithms.fields.PotentialField(self.match, name="AttackBehaviour")
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
    
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.id = self.robot.robot_id

        # upper goalpost 
        g_hgr = (self.field_h/2)+0.2-0.035
        # upper goalarea limit
        ga_hgr = self.field_h/2 + 0.4
    
        # lower goalpost
        g_lwr = (self.field_h/2)-0.2+0.035
        # lower goalarea limit
        ga_lwr = self.field_h/2 - 0.4

        def is_in_defensive_corner(m):
            if point_in_rect((m.ball.x, m.ball.y), (0, self.field_h-0.3, self.sa_w, 0.3)) or point_in_rect((m.ball.x, m.ball.y), (0, 0, self.sa_w, 0.3)):
                return True
            else:
                return False

        def is_in_defensive_area(m):
            if point_in_rect((m.ball.x, m.ball.y), (0, self.sa_w, self.sa_w, self.sa_h)):
                return True
            else:
                return False

        def sobra(m):    
            # tracks attacker position
            for r in self.match.robots:
                if r.strategy.name == self.attacker:
                    self.atk_x, self.atk_y = r.x, r.y
            
            if m.ball.x >= self.field_w/2 - 0.2:  
                # ball in top corner
                if point_in_rect((m.ball.x, m.ball.y), (self.field_w-self.sa_w, self.field_h/2 + 0.01, self.sa_w, self.field_h/2)):
                    x = self.field_w - self.sa_w - 0.25
                    y = self.field_h/2 - 0.1

                # ball in bottom corner
                elif point_in_rect((m.ball.x, m.ball.y), (self.field_w-self.sa_w, 0, self.sa_w, self.field_h/2)):
                    x = self.field_w - self.sa_w - 0.25
                    y = self.field_h/2 + 0.1

                else:
                    # elect the 'entity' that is closest to the middle of the field
                    if m.ball.x < self.atk_x:
                        ref_x = m.ball.x - 0.35
                        ref_y = m.ball.y
                    elif self.atk_x >= self.field_w/2 - 0.2:
                        ref_x = self.atk_x - 0.35
                        ref_y = self.atk_y
                    else:
                        ref_x = m.ball.x - 0.35
                        ref_y = m.ball.y

                    # higher third of the field's height
                    if m.ball.y > self.field_h * 2/3:
                        # if attacker comes from above
                        if self.atk_y < self.field_h * 2/3:
                            x = ref_x
                            y = self.field_h * 5/6
                        # if attacker comes from under
                        else:
                            x = ref_x
                            y = ref_y - 0.4

                    # middle third of the field's height
                    elif m.ball.y >= self.field_h*1/3 and m.ball.y <= self.field_h*2/3:
                        # if attacker comes from above or under
                        if self.atk_y < self.field_h * 1/3 or self.atk_y > self.field_h * 2/3:
                            x = ref_x
                            y = self.field_h/2
                        # if attacker is in the middle
                        else :
                            x = ref_x
                            y = ref_y
                    # lower third of the field's height
                    else:
                        # if attacker comes from above
                        if self.atk_y > self.field_h * 1/3:
                            x = ref_x
                            y = self.field_h * 1/6
                        # if attacker comes from under
                        else:
                            x = ref_x
                            y = ref_y + 0.4

            # defensive strategies
            elif m.ball.x < self.field_w/2 - 0.2 and not is_in_defensive_area(m):
                # defense top corner 
                if m.ball.y < self.field_h/2:
                    x = self.sa_w 
                    y = self.field_h - 0.15
                # defense bottom corner
                else:
                    x = self.sa_w
                    y = self.sa_y/2

            # ball in defensive goal area  
            else:
                if m.ball.y < self.field_h/2:
                    x = self.field_w/4
                    y = self.field_h - 0.10
                else:
                    x = self.field_w/4
                    y = self.sa_y/2
                    
            return (x, y)

        self.sobra.add_field(
            algorithms.fields.PointField(
                self.match,
                target = sobra,
                radius = 0.1,
                multiplier = VEL,
                decay = lambda x : x**6
            )
        )
        
        def future_point(m):
            # ball is on opponent's goal area
            if m.ball.x > self.field_w - self.sa_w - 0.05:
                # if ball is going down
                if m.ball.vy < 0:
                    y = m.ball.y - 0.08
                    t = (m.ball.y - y)/(m.ball.vy * (-1))
                    x = m.ball.x + m.ball.vx * t
                # if ball is going up
                elif m.ball.vy > 0:
                    y = m.ball.y + 0.08
                    t = (y - m.ball.y)/m.ball.vy
                    x = m.ball.x + m.ball.vx * t
                # if ball's vy = 0
                else:
                    x = m.ball.x
                    y = m.ball.y
            # ball isn't on opponent's goal area
            else:
                x = self.field_w - self.sa_w - 0.1
                y = self.field_h/2

            return x,y

        self.push.add_field(
            algorithms.fields.PointField(
                self.match,
                target = future_point,
                radius = 0.1,
                multiplier = lambda m: max(1, (m.ball.vx**2 + m.ball.vy**2)**0.5 + 0.3),
                decay = lambda x : x
            )
        )

        self.attack.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                radius = 0.1,
                multiplier = VEL,
                decay = lambda x : x
            )
        )

    def decide(self):

        for r in self.match.robots:
                if r.strategy.name == self.attacker:
                    self.atk_x, self.atk_y = r.x, r.y

        ball = self.match.ball
        behaviour = None
        self.theta = self.robot.theta

        def dist_to_ball(robot_y, robot_x):
            dist = ((robot_y - ball.y)**2 + (robot_x - ball.x)**2)**0.5
            return dist

        atk_dist = dist_to_ball(self.atk_y, self.atk_x)
        midf_dist = dist_to_ball(self.robot.y, self.robot.x)

        def is_atk_in_area():
            if self.atk_x >= self.field_w - self.sa_w and self.atk_y >= self.sa_y and self.atk_y <= self.sa_h + self.sa_y:
                return True
            else:
                return False

        if ball.vx < 0 and self.robot.x < ball.x and self.atk_x > ball.x and atk_dist > midf_dist:
            behaviour = self.attack
        
        elif (ball.x >= self.field_w - self.sa_w - 0.3 and ball.y > self.field_h/2-0.25 and 
              ball.y < self.field_h/2+0.25) and not is_atk_in_area():
            behaviour = self.push

        else:
            behaviour = self.sobra
    
        return behaviour.compute([self.robot.x, self.robot.y])