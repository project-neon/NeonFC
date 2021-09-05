import math
import algorithms
import controller
import numpy as np
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy

def get_ball_info(m):
    return (m.ball.vx, m.ball.vy, m.ball.x, m.ball.y)

class newGoalKeeper(Strategy):
    def __init__(self, match, plot_field=True):
        super().__init__(match, "MktGoalKeeper", controller=controller.TwoSidesLQR)

    def start(self, robot=None):
        super().start(robot=robot)

        self.restrict = algorithms.fields.PotentialField(self.match, name="RestrictBehaviour")

        self.project = algorithms.fields.PotentialField(self.match, name="ProjectBehaviour")

        self.path = algorithms.fields.PotentialField(self.match, name="PathBehaviour")

        self.kalm = algorithms.fields.PotentialField(self.match, name="KalmBehaviour")

        self.redeploy = algorithms.fields.PotentialField(self.match, name="RedeployBehaviour")

        #small area x, y, width and height
        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        g_hgr = (self.field_h/2)+0.2
        g_lwr = (self.field_h/2)-0.2

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (self.sa_w, self.sa_y+self.sa_h/2),
                theta = math.pi/2,
                line_size = 1.3/2,
                line_dist = 0.15,
                line_dist_max = 0.3,
                line_dist_single_side = True,
                inverse = True,
                multiplier = 0.75,
                decay = lambda x : 1
            )
        )

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (0, self.sa_y+self.sa_h/2),
                theta = 3*math.pi/2,
                line_size = self.sa_h/2,
                line_dist = 0.075,
                line_dist_max = 0.1,
                line_dist_single_side = True,
                inverse = True,
                multiplier = 0.75,
                decay = lambda x : 1
            )
        )

        self.project.add_field(self.restrict)

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
                multiplier = 0.7,
                decay = lambda x : x
            )
        )

        self.path.add_field(self.restrict)
        
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

            #segue a cordenada y se a bola está no meio do campo indo para a lateral
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
                multiplier = 0.7,
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

        behaviour = None

        theta = self.robot.theta

        #print(get_ball_info(self.match))
        if  (self.robot.x < self.sa_w-0.04 and self.robot.x > 0.02 and self.robot.y > self.sa_y 
             and self.robot.y < self.sa_y + self.sa_h):

            if (theta >= -1.65 and theta <= -1.485) or (theta >= 1.485 and theta <= 1.65):

                if self.match.ball.x < self.field_w/2 and self.match.ball.vx > 0:
                    behaviour = self.project
                    
                elif self.match.ball.x < self.field_w/2: #and self.match.ball.vx < 0:
                    behaviour = self.path

                elif self.match.ball.x >= self.field_w/2:
                    behaviour = self.kalm

                else:
                    behaviour = self.restrict

            else:

                if self.match.ball.x >= self.field_w/2:
                    behaviour = self.redeploy

                else:
                    behaviour = self.path

        else:
            behaviour = self.redeploy

        #return super().decide(self.path)

        return behaviour.compute([self.robot.x, self.robot.y])
    
    def spin(self):
        if (self.match.ball.y - self.robot.y) > 0:
            return 120, -120
        return -120, 120

    def spinning_time(self):
        dist_to_ball = np.linalg.norm(
            np.array([self.robot.x, self.robot.y]) - 
            np.array([self.match.ball.x, self.match.ball.y])
        )
        if dist_to_ball <= 0.12:
            return True
        return False

    def update(self):
        if self.spinning_time():
            return self.spin()
        return self.controller.update()
