import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

class newGoalKeeper(Strategy):
    def __init__(self, match, name="Higuita"):
        super().__init__(match, name, controller=controller.TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")

        self.kalm = algorithms.fields.PotentialField(self.match, name="KalmBehaviour")

        self.redeploy = algorithms.fields.PotentialField(self.match, name="RedeployBehaviour")
        
        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        self.category = self.match.category
        
        #trave superior do gol
        g_hgr = (self.field_h/2)+0.2

        #trave inferior do gol
        g_lwr = (self.field_h/2)-0.2

        def follow_ball(m):
            if m.ball.y > g_hgr:
                return (self.sa_w/2, g_hgr)
            elif m.ball.y < g_lwr:
                return (self.sa_w/2, g_lwr)
            else:
                return (self.sa_w/2, m.ball.y)

        self.project.add_field(
            algorithms.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = self.sa_w/2,
                line_dist = 0.1,
                line_dist_max = 0.7,
                multiplier = 1,
                decay = lambda x : x
            )
        )
        
        #cria a area de cobertura do goleiro quando esta nos cantos do gol
        def get_cover_area(robot, side):
            robot_w = robot_h = 0.075
            if side == "inf":
                robot_ext_x, robot_ext_y = (self.sa_w/2+robot_w/2, g_lwr+robot_h/2)
                cover_func = lambda x : ((robot_ext_y-g_hgr)/robot_ext_x)*x + g_hgr
                return cover_func

            elif side == "sup":
                robot_ext_x, robot_ext_y = (self.sa_w/2+robot_w/2, g_hgr-robot_h/2)
                cover_func = lambda x : ((robot_ext_y-g_lwr)/robot_ext_x)*x + g_lwr
                return cover_func

        #retorna a intersecção entre a linha do goleiro e uma curva de bezier,
        # que representa uma possível trajetória da bola
        def bezier_intersec(m, dir):
            t = 0
            if dir == "rise":
                y_med = (g_hgr+m.ball.y)/2
            elif dir == "drop":
                y_med = (g_lwr+m.ball.y)/2
            while t <= 1:
                y = (m.ball.vy/m.ball.vx)*(self.sa_w-m.ball.x) + m.ball.y
                b_x = ((1-t)**2)*m.ball.x + 2*(1-t)*t*self.sa_w
                b_y = ((1-t)**2)*m.ball.y + 2*(1-t)*t*y + (t**2)*y_med
                if 0.05 > round(b_x, 3) and round(b_x, 3) < 0.1:
                    return b_y
                t += 0.01
            print(f"False: {m.ball.vy/m.ball.vx}")
            return self.robot.y

        #retorna a posição em que o campo deve ser criado, para que a bola seja defendida
        def get_def_spot(m):
            x = self.sa_w/2

            if m.ball.vx == 0:
                if m.ball.y > g_hgr:
                    y = g_hgr
                    return (x, y)
                elif m.ball.y < g_lwr:
                    y = g_lwr
                    return (x, y)
                else:
                    y = m.ball.y
                    return (x, y)

            y = (m.ball.vy/m.ball.vx)*(x-m.ball.x) + m.ball.y

            g_cvr_sup = get_cover_area(self.robot, "sup")(m.ball.x)
            g_cvr_inf = get_cover_area(self.robot, "inf")(m.ball.x)

            #trava o goleiro na lateral do gol caso a bola esteja no escanteio ou 
            #acima da linha do gol e indo para o escanteio
            if (m.ball.y > g_cvr_sup and m.ball.y > g_hgr) or (m.ball.y > g_hgr and y > g_hgr):
                y = g_hgr
                return (x, y)

            elif (m.ball.y < g_cvr_inf and m.ball.y < g_lwr) or (m.ball.y < g_lwr and y < g_lwr):
                y = g_lwr
                return (x, y)

            #bloqueia uma possivel trajetoria da bola se ela esta no meio do campo indo para a lateral
            if y > g_hgr and g_lwr < m.ball.y < g_hgr:
                y =  max(bezier_intersec(m, "rise"), self.robot.y, m.ball.y)
                if y > g_hgr: y = g_hgr
                return (x, y)
                
            elif y < g_lwr and g_hgr > m.ball.y > g_lwr:
                y = min(bezier_intersec(m, "drop"), self.robot.y, m.ball.y)
                if y < g_lwr: y = g_lwr
                return (x, y)

            mid_field_h = self.field_h/2

            #caso a bola esteja entre o escanteio e o meio do campo indo para uma dos escanteios,
            #o robo defende a trajetoria entre a bola e o gol
            if g_cvr_inf < m.ball.y < g_lwr and y > self.robot.y:
                y = ((m.ball.y-mid_field_h+0.1)/m.ball.x)*x + mid_field_h - 0.1
                return (x, y)

            elif g_cvr_sup > m.ball.y > g_hgr and y < self.robot.y:
                y = ((m.ball.y-mid_field_h-0.1)/m.ball.x)*x + mid_field_h + 0.1
                return (x, y)

            return (x, y)

        self.path.add_field(
            algorithms.fields.LineField(
                self.match,
                target = get_def_spot,
                theta = 0,
                line_size = self.sa_w/2,
                line_dist = 0.1,
                line_dist_max = 0.7,
                multiplier = 1,
                decay = lambda x : x
            )
        )

        # permanece no centro da área
        self.kalm.add_field(
            algorithms.fields.LineField(
                self.match,
                target = lambda m: (self.sa_w/2, self.field_h/2),
                theta = 0,
                line_size = self.sa_w/2,
                line_dist = 0.1,
                line_dist_max = self.sa_h,
                decay = lambda x: x,
                multiplier = 0.7,
            )
        )

        self.redeploy.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target = (self.sa_w/2, self.field_h/2),
                radius = 0.00001,
                radius_max = self.field_w,
                clockwise = True,
                decay = lambda x: 1,
                multiplier = 0.7
            )
        )

    def decide(self):

        self.theta = self.robot.theta

        behaviour = None
        if (self.robot.x <= self.sa_w - 0.0375  and self.robot.x > 0.0375 and self.robot.y >= self.sa_y 
            and self.robot.y <= self.sa_y + self.sa_h):

            if self.match.ball.x < self.field_w/2:

                if self.match.ball.vx > 0:
                    behaviour = self.project
                
                else:
                    behaviour = self.path

            else:
                behaviour = self.kalm
        
        else:
            behaviour = self.redeploy

        return behaviour.compute([self.robot.x, self.robot.y])

    def spin(self):
        dist_to_ball = math.sqrt(
            (self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2
        )
        if dist_to_ball <= 0.12:
            if (self.match.ball.y - self.robot.y) > 0:
                return 120, -120
            else:
                return -120, 120
        else:
            if self.match.team_color.upper() == "BLUE":
                w = ((self.theta**2)**0.5 - 1.5708) * 20
            else:
                w = ((((self.theta**2)**0.5 - 4.71239)**2)**0.5) * 15
        return -w, w

    def spinning_time(self):
        dist_to_ball = math.sqrt(
            (self.robot.x - self.match.ball.x)**2 + (self.robot.y - self.match.ball.y)**2
        )
        if dist_to_ball > 0.12:
            if (self.robot.x <= self.sa_w-0.0375  and self.robot.x > 0.0375  and self.robot.y >= self.sa_y 
                and self.robot.y <= self.sa_y + self.sa_h):
                if self.match.team_color.upper() == "BLUE":
                    if ((self.theta >= -1.61 and self.theta <= -1.54) or (self.theta >= 1.54 and self.theta <= 1.61)):
                        return False
                    else:
                        return True
                else:
                    theta = self.theta*180/math.pi
                    if (theta >= 87 and theta <= 93) or (theta >= 267 and theta <= 273):
                        return False
                    else:
                        return True
            else: 
                return False
        else:
            if self.category == "5v5":
                return False
            else:
                return True

    def update(self):
        if self.spinning_time():
            return self.spin()
        return self.controller.update()