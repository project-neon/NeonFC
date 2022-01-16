import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from commons.math import point_in_rect


class MidFielderkkkkk(Strategy):
    def __init__(self, match, has_defensive_behaviour=True):
        super().__init__(match, "MidFielderkkkk", controller=controller.TwoSidesLQR)
    
    def start(self, robot=None):
        super().start(robot=robot)

        for r in self.match.robots:
            if r.strategy.name in ["UFV-Attacker", "shooter"]:
                self.atk_x, self.atk_y = r.x, r.y

        self.sobra = algorithms.fields.PotentialField(self.match, name="SobraBehaviour")

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")
    
        self.right_redeploy = algorithms.fields.PotentialField(self.match, name="RightRedeployBehaviour")
    
        self.left_redeploy = algorithms.fields.PotentialField(self.match, name="LeftRedeployBehaviour")    
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
    
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.id = self.robot.robot_id

        if self.match.category == "3v3":
            self.x = self.sa_w + 0.02
        else:
            self.x = self.sa_w + 0.225

        #trave superior do gol
        g_hgr = (self.field_h/2)+0.2-0.035
        ga_hgr = self.field_h/2 + 0.4
    
        #trave inferior do gol
        g_lwr = (self.field_h/2)-0.2+0.035
        ga_lwr = self.field_h/2 - 0.4

        def side_verifier(y):
            d = 0.05

            if self.name == "LeftDefender":
                y += d
            elif self.name == "RightDefender":
                y -= d
            return y

        # novo (2k22)
        def sobra(m):
            for r in self.match.robots:
                if r.strategy.name in ["UFV-Attacker", "shooter", "Idle"]:
                    self.atk_x, self.atk_y = r.x, r.y
            # top corner
            if point_in_rect((m.ball.x, m.ball.y), (self.field_w-self.sa_w, self.field_h/2, self.sa_w, self.field_h/2)):
                #if point_in_rect((self.atk_x, self.atk_y), (self.field_w-self.sa_w, self.field_h-0.3, self.sa_w, 0.3)):
                x = self.atk_x - 0.175
                y = self.atk_y - 0.3
                """else:
                    x = self.atk_x - 0.3
                    y = m.ball.y"""
            # bottom corner
            elif point_in_rect((m.ball.x, m.ball.y), (self.field_w-self.sa_w, 0, self.sa_w, self.field_h/2)):
                #if point_in_rect((self.atk_x, self.atk_y), (self.field_w-self.sa_w, 0, self.sa_w, 0.3)):
                x = self.atk_x - 0.175
                y = self.atk_y + 0.3
                """else:
                    x = self.atk_x - 0.3
                    y = m.ball.y"""
            # top side
            elif point_in_rect((m.ball.x, m.ball.y), (self.field_w/2, self.field_h-0.3, self.field_w/2 - self.sa_w, 0.3)):
                if point_in_rect((self.atk_x, self.atk_y), (self.field_w/2, self.field_h-0.3, self.field_w/2 - self.sa_w, 0.3)):
                    x = self.atk_x - 0.3
                    y = self.atk_y - 0.15
                else:
                    x = self.atk_x - 0.3
                    y = m.ball.y
            # bottom side
            elif point_in_rect((m.ball.x, m.ball.y), (self.field_w/2, 0, self.field_w/2 - self.sa_w, 0.3)):
                if point_in_rect((self.atk_x, self.atk_y), (self.field_w/2, 0, self.field_w/2 - self.sa_w, 0.3)):
                    x = self.atk_x - 0.3
                    y = self.atk_y + 0.15
                else:
                    x = self.atk_x - 0.3
                    y = m.ball.y
            # middle
            else:
                x = self.atk_x - 0.3
                y = m.ball.y    

            return x, y

        def follow_ball(m):
            if m.ball.y > g_hgr:
                y = side_verifier(g_hgr)
                return (self.x, y)
            elif m.ball.y < g_lwr:
                y = side_verifier(g_lwr)
                return (self.x, y)
            else:
                y = side_verifier(m.ball.y)
                return (self.x, y)

        self.project.add_field(
            algorithms.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = 0.7,
                multiplier = 0.7,
                decay = lambda x : x
            )
        )
        
        #retorna a posição em que o campo deve ser criado, para que a bola seja defendida
        def get_mid_value(a, b, c):
            return max(min(a,b), min(max(a,b),c))

        #retorna a posição em que o campo deve ser criado, para que a bola seja defendida
        def get_def_spot(m):
            x = self.x
            
            if m.ball.vx == 0:
                if m.ball.y > g_hgr:
                    y = side_verifier(g_hgr)
                    return (x, y)
                elif m.ball.y < g_lwr:
                    y = side_verifier(g_lwr)
                    return (x, y)
                else:
                    y = side_verifier(m.ball.y)
                    return (x, y)

            if m.ball.y > ga_hgr:
                y = side_verifier(g_hgr)
                return (x, y)
            elif m.ball.y < ga_lwr:
                y = side_verifier(g_lwr)
                return (x, y)
            else:
                if m.ball.x > 0.4:
                    gk_y = self.match.robots[0].y

                    if self.name == "RightDefender":
                        gk_inf = gk_y-0.075/2
                        m_inf = (gk_inf+g_lwr)/2
                        y = ( (m.ball.y-m_inf)/m.ball.x)*x + m_inf
                        y = get_mid_value(y, side_verifier(g_lwr), side_verifier(g_hgr))
                        return (x, y)
                    elif self.name == "LeftDefender":
                        gk_sup = gk_y+0.075/2
                        m_sup = (gk_sup+g_hgr)/2
                        y = ( (m.ball.y-m_sup)/m.ball.x)*x + m_sup
                        y = get_mid_value(y, side_verifier(g_lwr), side_verifier(g_hgr))
                        return (x, y)

                y = ( (m.ball.y-(self.field_h/2) )/m.ball.x)*x + self.field_h/2
                y = get_mid_value(side_verifier(y), side_verifier(g_lwr), side_verifier(g_hgr))
                return (x, y)


        self.path.add_field(
            algorithms.fields.LineField(
                self.match,
                target = get_def_spot,
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = self.field_h,
                multiplier = 1,
                decay = lambda x : x
            )
        )

        self.sobra.add_field(
            algorithms.fields.PointField(
                self.match,
                target = sobra,
                radius = 0.1,
                multiplier = 1,
                decay = lambda x : x**6
            )
        )
        
        self.left_redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.x, self.field_h/2 + 0.2),
                radius = 0,
                radius_max = self.field_w,
                clockwise = False,
                decay = lambda x: 1,
                multiplier = 1
            )
        )

        self.right_redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.x, self.field_h/2 - 0.2),
                radius = 0,
                radius_max = self.field_w,
                clockwise = True,
                decay = lambda x: 1,
                multiplier = 1
            )
        )
    
    def decide(self):
        ball = self.match.ball
        self.theta = self.robot.theta
        behaviour = None
        self.maneuver = "yep"
        dist_atk_ball = math.sqrt((ball.x - self.atk_x)**2 + (ball.y - self.atk_y)**2)

        if ball.x < self.field_w/2:
            if (self.robot.x >= self.x-0.01) and (self.robot.x < self.x + 0.025):
                   
                if self.match.ball.x > 0.225:
                    behaviour = self.path
                else:
                    behaviour = self.project

            else:
                if ball.y > self.field_h/2:
                    behaviour = self.right_redeploy
                else:
                    behaviour = self.left_redeploy

        else:
            self.maneuver = "nope"
            behaviour = self.sobra

    
        return behaviour.compute([self.robot.x, self.robot.y])

    def spin(self):
        if self.match.team_color.upper() == "BLUE":
            w = ((self.theta**2)**0.5 - 1.5708) * 20
        else:
            w = ((((self.theta**2)**0.5 - 4.71239)**2)**0.5) * 15
        return -w, w
    
    def spinning_time(self):
        if self.maneuver == "yep":
            if (self.robot.x > self.x-0.01 and self.robot.x < self.x + 0.02):
                if self.match.team_color.upper() == "BLUE":
                    if ((self.theta >= -1.61 and self.theta <= -1.54) or (self.theta >= 1.54 and self.theta <= 1.61)):
                        return False
                    else:
                        return True
                else:
                    theta = self.theta*180/math.pi
                    if ((theta >= 87 and theta <= 93) or (theta >= 267 and theta <= 273)):
                        return False
                    else:
                        return True
            else:
                return False
        else:
            return False

    def update(self):
        if self.spinning_time():
            return self.spin()
        return self.controller.update()
                     