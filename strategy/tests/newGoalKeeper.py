import math
import algorithms
import controller
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

        #small area x, y, width and height
        sa_x, sa_y, sa_w, sa_h = self.match.game.field.get_small_area("defensive")

        field_w, field_h = self.match.game.field.get_dimensions()

        g_hgr = (field_h/2)+0.2
        g_lwr = (field_h/2)-0.2

        self.restrict.add_field(
            algorithms.fields.LineField(
                self.match,
                target = (sa_w, sa_y+sa_h/2),
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
                target = (0, sa_y+sa_h/2),
                theta = 3*math.pi/2,
                line_size = sa_h/2,
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
                return (sa_w/2, g_hgr)
            elif m.ball.y < g_lwr:
                return (sa_w/2, g_lwr)
            else:
                return (sa_w/2, m.ball.y)

        self.project.add_field(
            algorithms.fields.LineField(
                self.match,
                target = follow_ball,
                theta = 0,
                line_size = sa_w/2,
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
                robot_ext_x, robot_ext_y = (sa_w/2+robot_w/2, g_lwr+robot_h/2)
                cover_func = lambda x : ((robot_ext_y-g_hgr)/robot_ext_x)*x + g_hgr
                return cover_func

            elif side == "sup":
                robot_ext_x, robot_ext_y = (sa_w/2+robot_w/2, g_hgr-robot_h/2)
                cover_func = lambda x : ((robot_ext_y-g_lwr)/robot_ext_x)*x + g_lwr
                return cover_func


        def get_def_spot(m):
            x = sa_w/2

            if m.ball.vy==0 or m.ball.vx==0:
                return (x, field_h/2)

            y = (m.ball.vy/m.ball.vx)*(x-m.ball.x) + m.ball.y

            g_cvr_sup = get_cover_area(self.robot, "sup")(m.ball.x)
            g_cvr_inf = get_cover_area(self.robot, "inf")(m.ball.x)

            #trava o goleiro na lateral do gol caso a bola esteja no escanteio ou 
            #acima da linha do gol e indo para o escanteio
            if m.ball.y > g_cvr_sup or (m.ball.y > g_hgr and y > g_hgr):
                y = g_hgr
                return (x, y)

            elif m.ball.y < g_cvr_inf or (m.ball.y < g_lwr and y < g_lwr):
                y = g_lwr
                return (x, y)

            #segue a cordenada y se a bola está no meio do campo indo para a lateral
            if y > g_hgr and g_lwr < m.ball.y < g_hgr:
                y =  m.ball.y
                return (x, y)
                
            elif y < g_lwr and g_hgr > m.ball.y > g_lwr:
                y = m.ball.y
                return (x, y)

            mid_field_h = field_h/2

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
                line_size = sa_w/2,
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
                target = lambda m: (0.075, field_h/2),
                theta = 0,
                line_size = sa_w/2,
                line_dist = 0.1,
                line_dist_max = 0.7,
                decay = lambda x: x,
                multiplier = 0.7,
            )
        )

    def decide(self):

        behaviour = None

        #print(get_ball_info(self.match))

        if self.match.ball.x < 0.750: #and self.match.ball.vx < 0:
            behaviour = self.path

        elif self.match.ball.x < 0.750 and self.match.ball.vx > 0:
            behaviour = self.project

        elif self.match.ball.x >= 0.750:
            behaviour = self.kalm

        else:
            behaviour = self.restrict

        #return super().decide(self.path)

        return behaviour.compute([self.robot.x, self.robot.y])
