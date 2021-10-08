import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy


class newMidFielder(Strategy):
    def __init__(self, match, plot_field=False):
        super().__init__(match, "NeonDesarmaRindokkkkkkkkk", controller=controller.TwoSidesLQR)
    
    def start(self, robot=None):
        super().start(robot=robot)

        self.intercept = algorithms.fields.PotentialField(self.match, name="InterceptBehaviour")

        self.push = algorithms.fields.PotentialField(self.match, name="PushBehaviour")

        self.sobra = algorithms.fields.PotentialField(self.match, name="SobraBehaviour")

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")
    
        self.right_redeploy = algorithms.fields.PotentialField(self.match, name="RightRedeployBehaviour")
    
        self.left_redeploy = algorithms.fields.PotentialField(self.match, name="LeftRedeployBehaviour")    
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
    
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.x = self.sa_w + 0.4
        
        #trave superior do gol
        g_hgr = (self.field_h/2)+0.2-0.0375
    
        #trave inferior do gol
        g_lwr = (self.field_h/2)-0.2+0.0375

        def intercept(m):
           if m.ball.y > self.field_h/2:
               x = (m.ball.x + m.ball.vx * (1/60)) - 0.1
               y = (m.ball.y + m.ball.vy * (1/60))- 0.3
           else:
               x = (m.ball.y + m.ball.vy * (1/60)) - 0.1
               y = (m.ball.y + m.ball.vy * (1/60)) + 0.3
 
           if x < self.sa_w*2:
               x = self.sa_w + 0.01
               if m.ball.y > self.field_h/2:
                   y = self.g_hgr - 0.0375
               elif m.ball.y < self.field_h/2:
                   y = self.g_lwr + 0.0375
               else:
                   y = m.ball.y
 
           return x,y

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

            if x < self.sa_w*2: 
                x = self.sa_w + 0.01
                if m.ball.y > self.sa_y + self.sa_h:
                    y = g_hgr
                elif m.ball.y < self.sa_y:
                    y = g_lwr
                else:
                    y = m.ball.y

            return x,y

        def follow_ball(m):
            if m.ball.y > g_hgr:
                y = g_hgr
                return (self.x, y)
            elif m.ball.y < g_lwr:
                y = g_lwr
                return (self.x, y)
            else:
                y = m.ball.y
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
        def get_def_spot(m):
            x = self.x
    
            if m.ball.vx == 0:
                return x, m.ball.y
            
            else:
                t = (x - m.ball.x)/m.ball.vx
                y = m.ball.y + m.ball.vy * t
    
                if m.ball.vx > 0 and m.ball.y > self.field_h/2:
                    return x, m.ball.y - 0.3
    
                elif m.ball.vx > 0 and m.ball.y <= self.field_h/2:
                    return x, m.ball.y + 0.3
                
                else:
                    return x, y

            
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

        self.intercept.add_field(
           algorithms.fields.PointField(
               self.match,
               target = intercept,
               radius = 0.1,
               decay = lambda x: x,
               multiplier = 0.7
           )
       )

        
        self.left_redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.x, self.sa_h+self.sa_y - 0.07),
                radius = 0,
                radius_max = self.field_w,
                clockwise = False,
                decay = lambda x: 1,
                multiplier = 0.8
            )
        )

        self.right_redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.x, self.sa_y + 0.07),
                radius = 0,
                radius_max = self.field_w,
                clockwise = True,
                decay = lambda x: 1,
                multiplier = 0.8
            )
        )
    
    def decide(self):
        ball = self.match.ball
        self.theta = self.robot.theta
        behaviour = None
        self.behaviour = 'prende'

        if ball.vx < 0 and ball.x < self.field_w - self.sa_w - 0.3:
            if ((self.robot.x >= self.sa_w+0.02) and (self.robot.x < self.field_w/2)
                and (ball.y < self.sa_y + self.sa_h) and (ball.y > self.sa_y)):
                    
                    if self.match.ball.x > self.robot.x:
                        behaviour = self.path
                    else:
                        behaviour = self.project
            elif ball.y >= self.sa_y + self.sa_h and ball.y <= self.sa_y:
                behaviour = self.intercept
        
            else:
                if ball.y > self.field_h/2:
                    behaviour = self.left_redeploy
                else:
                    behaviour = self.right_redeploy
        elif (ball.x >= self.field_w - self.sa_w - 0.3 and ball.y > 0.35 and ball.y < 0.85):
            self.behaviour = 'solta'
            behaviour = self.push
        else:
            self.behaviour = 'solta'
            behaviour = self.sobra

    
        return behaviour.compute([self.robot.x, self.robot.y])

    def spin(self):
        if self.match.team_color.upper() == "BLUE":
            w = ((self.theta**2)**0.5 - 1.5708) * 20
        else:
            w = ((((self.theta**2)**0.5 - 4.71239)**2)**0.5) * 15
        return -w, w
    
    def spinning_time(self):
        if self.behaviour == 'prende':
            if (self.robot.x > self.sa_w+0.01 and self.robot.x < self.sa_w + 0.3):
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