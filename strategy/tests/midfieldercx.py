import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from commons.math import point_in_rect


class MidFielder2(Strategy):
    def __init__(self, match, name = 'MidFielder2'):
        self.ctrl_params = {"l": 0.07}
        super().__init__(match,
            name=name,
            controller=controller.TwoSidesLQR,
            controller_kwargs=self.ctrl_params
        )


    
    def start(self, robot=None):
        super().start(robot=robot)

        for r in self.match.robots:
            if r.strategy.name in ["UFV-Attacker", "shooter", "AtacanteMeu"]:
                self.atk_x, self.atk_y = r.x, r.y

        VEL = 1

        self.sobra = algorithms.fields.PotentialField(self.match, name="SobraBehaviour")

        self.sobra_protect = algorithms.fields.PotentialField(self.match, name="SobraProtectBehaviour")

        self.push = algorithms.fields.PotentialField(self.match, name="PushBehaviour")

        self.avoid_area = algorithms.fields.PotentialField(self.match, name="AvoidAreaBehaviour")
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
    
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.id = self.robot.robot_id

        #trave superior do gol
        g_hgr = (self.field_h/2)+0.2-0.035
        ga_hgr = self.field_h/2 + 0.4
    
        #trave inferior do gol
        g_lwr = (self.field_h/2)-0.2+0.035
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

        # novo (2k22)
        def sobra(m):    

            for r in self.match.robots:
                if r.strategy.name in ["UVF-Attacker", "shooter", "AtacanteMeu"]:
                    self.atk_x, self.atk_y = r.x, r.y
            
            if m.ball.x >= self.field_w/2:  
                # top corner
                if point_in_rect((m.ball.x, m.ball.y), (self.field_w-self.sa_w, self.field_h/2, self.sa_w, self.field_h/2)):
                    x = self.field_w - self.sa_w - 0.2
                    y = self.field_h/2 + 0.1

                # bottom corner
                elif point_in_rect((m.ball.x, m.ball.y), (self.field_w-self.sa_w, 0, self.sa_w, self.field_h/2)):
                    x = self.field_w - self.sa_w - 0.2
                    y = self.field_h/2 - 0.1

                else:
                    # terço maior do campo
                    if m.ball.y > self.field_h * 2/3:
                        # se o atacante vem de baixo
                        if self.atk_y < self.field_h * 2/3:
                            x = min(m.ball.x, self.atk_x) - 0.4
                            y = self.field_h * 5/6
                        # se o atacante vem de cima
                        else:
                            x = min(m.ball.x, self.atk_x) - 0.4
                            y = m.ball.y - 0.3

                    # segundo terço do campo
                    elif m.ball.y > self.field_h*1/3 and m.ball.y < self.field_h*2/3:
                        # se o atacante vem de baixo ou de cima 
                        if self.atk_y < self.field_h * 1/3 or self.atk_y > self.field_h * 2/3:
                            x = min(m.ball.x, self.atk_x) - 0.4
                            y = self.field_h/2
                        # se o atacante esta no meio
                        else :
                            x = min(m.ball.x, self.atk_x) - 0.4
                            y = m.ball.y
                    # primeiro terço do campo
                    else:
                        # se o atacante vem de cima
                        if self.atk_y > self.robot.y:
                            x = min(m.ball.x, self.atk_x) - 0.4
                            y = self.field_h * 1/6
                        # se o atacante vem de baixo
                        else:
                            x = min(m.ball.x, self.atk_x) - 0.4
                            y = m.ball.y + 0.3
            # ????
            elif m.ball.x < self.field_w/2 and not is_in_defensive_area(m):
                    # defense top corner or bottom corner
                    # if is_in_defensive_corner(m):
                if m.ball.y < self.field_h/2:
                    x = self.sa_w 
                    y = self.field_h - 0.15
                    
                else:
                    x = self.sa_w
                    y = self.sa_y/2
                   
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
            if m.ball.vy != 0:
                self.future_y = self.field_h/2
                t = ((self.future_y - m.ball.y)**2)**0.5/m.ball.vy
                self.future_x = m.ball.x + m.ball.vx * t   
                return (self.future_x-0.04, self.future_y)
            else:
                return (self.robot.x, self.robot.y)

        self.push.add_field(
            algorithms.fields.PointField(
                self.match,
                target = future_point,
                radius = 0.1,
                multiplier = VEL,
                decay = lambda x : x
            )
        )

        # Avoid defensive area
        self.sobra_protect.add_field(
            algorithms.fields.LineField(
                self.match,
                target = [0, self.field_h/2],                                                                                                                                                                                                                                                                                                                                          
                theta = math.pi/2,
                line_size = self.sa_h/2,
                line_dist = 0.23,
                line_dist_max = 0.23,
                decay = lambda x: 1,
                multiplier = -2
            )
        )

        self.sobra_protect.add_field(
            algorithms.fields.PointField(
                self.match,
                target = sobra,
                radius = 0.1,
                multiplier = VEL,
                decay = lambda x : x**6
            )
        )


    # melhorar posicionamento do atacante e meio campo
    def decide(self):
        ball = self.match.ball
        behaviour = None
        self.theta = self.robot.theta

        def is_atk_in_area():
            if self.atk_x >= self.field_w - self.sa_w and self.atk_y >= self.sa_y and self.atk_y <= self.sa_h + self.sa_y:
                return True
            else:
                return False
        
        if (ball.x >= self.field_w - self.sa_w - 0.3 and ball.y > self.field_h/2-0.25 and 
              ball.y < self.field_h/2+0.25) and not is_atk_in_area():
            behaviour = self.push

        elif self.robot.x < self.field_w/2 and ball.x >= self.field_w/2:
            behaviour = self.sobra_protect

        else:
            behaviour = self.sobra
    
        return behaviour.compute([self.robot.x, self.robot.y])

    def spin(self):

        if point_in_rect((self.robot.x, self.robot.y), (self.sa_w - 0.0375, self.field_h - 0.1875, 0.075, 0.075)):
            w = ((self.theta**2)**0.5 - 0.785) * 20
        else:
            w = ((self.theta**2)**0.5 + 0.785) * 5

        return -w, w

    def spinning_time(self):
      
        if point_in_rect((self.robot.x, self.robot.y), (self.sa_w - 0.0375, self.field_h - 0.1875, 0.075, 0.075)):
            if (self.theta < -0.7504 and self.theta > -0.8203):
                return False
            else:
                return True

        else: 
            return False
    
    def update(self):
        if self.spinning_time():
            return self.spin()
        return self.controller.update()

                     