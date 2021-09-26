import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
    
class Defender(Strategy):
    def __init__(self, match, side, plot_field=False):
        super().__init__(match, side+"Defender", controller=controller.TwoSidesLQR)
        self.name = side+"Defender"
    
    def start(self, robot=None):
        super().start(robot=robot)

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")
    
        self.kalm = algorithms.fields.PotentialField(self.match, name="KalmBehaviour")
    
        self.redeploy = algorithms.fields.PotentialField(self.match, name="RedeployBehaviour")
    
        self.alert = algorithms.fields.PotentialField(self.match, name="AlertBehaviour")
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
    
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.x = self.sa_w + 0.075
        
        #trave superior do gol
        g_hgr = (self.field_h/2)+0.2
        sa_hgr = self.field_h/2 + self.sa_h/2
        ga_hgr = self.field_h/2 + 0.4
    
        #trave inferior do gol
        g_lwr = (self.field_h/2)-0.2
        sa_lwr = self.field_h/2 - self.sa_h/2
        ga_lwr = self.field_h/2 - 0.4

        def side_verifier(y):
            d = 0.05

            if self.name == "LeftDefender":
                y += d
            elif self.name == "RightDefender":
                y -= d
            return y

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
                multiplier = 0.7,
                decay = lambda x : x
            )
        )
    
        # permanece no centro da área
        self.kalm.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (self.x, self.field_h/2 - 0.09),
                theta = 0,
                line_size = self.field_h - self.sa_w,
                line_dist = 0.1,
                line_dist_max = self.field_h/2,
                decay = lambda x: x,
                multiplier = 0.7,
            )
        )
        
        self.redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.x, self.field_h/2),
                radius = 0.00001,
                radius_max = self.field_w,
                clockwise = True,
                decay = lambda x: 1,
                multiplier = 0.7
            )
        )
    
        self.alert.add_field(
            algorithms.fields.PointField(
                self.match,
                target = get_def_spot,
                radius = 0.1, # 30cm
                decay = lambda x: x**2,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.7
            )
        )
    
    def decide(self):
    
        self.theta = self.robot.theta
    
        behaviour = None
    
        if (self.robot.x >= self.sa_w + 0.0375) and (self.robot.x < self.field_w/2 - 0.0375):
    
            if (self.theta >= -1.67 and self.theta <= -1.47) or (self.theta >= 1.47 and self.theta <= 1.67):
                
                if self.match.ball.x > 0.225:
                    behaviour = self.path
                else:
                    behaviour = self.project
    
            else:
                #if self.match.ball.x >= self.field_w/2 + 0.25:
                    #behaviour = self.redeploy
    
                #else:
                    behaviour = self.alert
    
        else:
            behaviour = self.redeploy
    
        return behaviour.compute([self.robot.x, self.robot.y])

    """def spin(self):
        if (self.match.ball.y - self.robot.y) > 0:
            return 120, -120
        return -120, 120

    def spinning_time(self):
        self.dist_to_ball = np.linalg.norm(
            np.array([self.robot.x, self.robot.y]) - 
            np.array([self.match.ball.x, self.match.ball.y])
        )
        if (self.dist_to_ball <= 0.12 and self.match.ball.vx < 0):
            return True
        return False

    def update(self):
        if self.spinning_time():
            return self.spin()
        return self.controller.update()"""