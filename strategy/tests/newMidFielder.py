import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy


class newMidFielder(Strategy):
    def __init__(self, match, side, has_defensive_behaviour=True):
        super().__init__(match, side+"Defender", controller=controller.TwoSidesLQR)
        self.name = side+"Defender"

        self.has_defensive_behaviour = has_defensive_behaviour
    
    def start(self, robot=None):
        super().start(robot=robot)

        for r in self.match.robots:
            if r.strategy.name in ["UFV-Attacker", "shooter", "SpinnerAttacker"]:
                self.atk_x, self.atk_y = r.x, r.y

        self.triangulate = algorithms.fields.PotentialField(self.match, name="TriangulateBehaviour")

        self.push = algorithms.fields.PotentialField(self.match, name="PushBehaviour")

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
            self.x = self.sa_w + 0.04
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

        def future_point(m):
            if m.ball.vy != 0:
                self.future_y = self.field_h/2
                t = ((self.future_y - m.ball.y)**2)**0.5/m.ball.vy
                self.future_x = m.ball.x + m.ball.vx * t   
                return (self.future_x-0.04, self.future_y)
            else:
                return (self.robot.x, self.robot.y)

        def sobra(m):
            x = m.ball.x-0.45
            if m.ball.x < self.robot.x and m.ball.vx > 0:
                if m.ball.y > self.field_h/2:
                    y = m.ball.y - 0.3
                else:
                    y = m.ball.y + 0.3
            else:
                if m.ball.y > self.field_h/2:
                    y = self.field_h/2 + 0.1
                else:
                    y = self.field_h/2 - 0.1
                
                if self.field_h - y < 0.04:
                    y = self.field_h - 0.04
                elif y < 0.04:
                    y = 0.04

            return x,y

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

        def get_triang_pos(m):
            d_x = m.ball.x - self.atk_x
            d_y = m.ball.y - self.atk_y
            circ_r = math.sqrt( d_x**2 + d_y**2 )
            q = math.sqrt(d_x**2 + d_y**2)
            x_m = (m.ball.x + self.atk_x)/2
            y_m = (m.ball.y + self.atk_y)/2
            d = math.sqrt(circ_r**2 - (q/2)**2)
            x_c1 = x_m - d*(d_y/q)
            y_c1 = y_m + d*(d_x/q)
            x_c2 = x_m + d*(d_y/q)
            y_c2 = y_m - d*(d_x/q)

            if x_c1 < x_c2:
                x_c = x_c1
                y_c = y_c1
            else:
                x_c = x_c2
                y_c = y_c2
            
            if m.ball.vx == 0:
                tht = m.ball.vy
            else:
                tht = m.ball.vy/m.ball.vx
            
            a = 1 + (tht**2)
            b = 2*( tht*(m.ball.y - y_c - (tht*m.ball.x) ) - x_c)
            c = (x_c**2) - (circ_r**2) + (m.ball.y - y_c - (tht*m.ball.x) )**2
            
            try:
                x_t = (b*(-1) - math.sqrt(b**2 - 4*a*c) )/(2*a)
            except ValueError:
                x_t = self.field_w/2 + 0.2

            if x_t >= self.atk_x-0.1:
                x_t -= 0.2

            if x_t < self.field_w/2:
                x_t = self.field_w/2

            y = tht*(x_t - m.ball.x) + m.ball.y

            if y > self.field_h:
                y = self.field_h - 0.2
                x_t = ((y - m.ball.y)/tht) + m.ball.x
            elif y < 0:
                y = 0.2
                x_t = ((y - m.ball.y)/tht) + m.ball.x
            x_t -= 0.025
            return (x_t, y)

        self.triangulate.add_field(
            algorithms.fields.PointField(
                self.match,
                target = get_triang_pos,
                radius = 0.1,
                decay = lambda x: x**6,
                multiplier = 0.7
            )
        )

        self.path.add_field(
            algorithms.fields.LineField(
                self.match,
                target = get_def_spot,
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = self.field_h,
                multiplier = 0.7,
                decay = lambda x : x
            )
        )

        self.push.add_field(
            algorithms.fields.PointField(
                self.match,
                target = future_point,
                radius = 0.1,
                multiplier = 0.8,
                decay = lambda x : x
            )
        )

        self.sobra.add_field(
            algorithms.fields.PointField(
                self.match,
                target = sobra,
                radius = 0.1,
                multiplier = 0.7,
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
                multiplier = 0.65
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
                multiplier = 0.65
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
        
        elif self.match.ball.x > self.field_w/2 and self.match.ball.vx < 0 and dist_atk_ball>0.5:
            self.maneuver = "nope"
            behaviour = self.triangulate

        elif (ball.x >= self.field_w - self.sa_w - 0.3 and ball.y > self.field_h/2-0.25 and 
              ball.y < self.field_h/2+0.25):
            self.maneuver = "nope"
            behaviour = self.push
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