import math
import algorithms
from controller.simple_LQR import TwoSidesLQR, SimpleLQR
from strategy.BaseStrategy import Strategy

class Goalkeeper(Strategy):
    def __init__(self, match):
        super().__init__(match, "GoalkeeperIRON", controller=TwoSidesLQR)
        self.fps = self.match.game.vision._fps if self.match.game.vision._fps else 60

        self.g_left = 0.87
        self.g_right = 0.46

        self.gk_x = 0.12

    def start(self, robot=None):
        super().start(robot=robot)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

        # trave superior do gol
        g_hgr = (self.field_h / 2) + 0.185
        ga_hgr = g_hgr + 0.15

        # trave inferior do gol
        g_lwr = (self.field_h / 2) - 0.185
        ga_lwr = g_lwr - 0.15

        self.robot_w = self.robot_h = 0.075

        # cria a area de cobertura do goleiro quando esta nos cantos do gol
        def get_cover_area(robot, side):
            if side == "inf":
                robot_ext_x, robot_ext_y = (self.sa_w / 2 + self.robot_w / 2, g_lwr + self.robot_h / 2)
                cover_func = lambda x: ((robot_ext_y - g_hgr) / robot_ext_x) * x + g_hgr
                return cover_func

            elif side == "sup":
                robot_ext_x, robot_ext_y = (self.sa_w / 2 + self.robot_w / 2, g_hgr - self.robot_h / 2)
                cover_func = lambda x: ((robot_ext_y - g_lwr) / robot_ext_x) * x + g_lwr
                return cover_func

        # retorna a posição em que o campo deve ser criado, para que a bola seja defendida
        def get_def_spot(m):
            x = self.gk_x + 0.04

            if m.ball.x < (self.sa_w / 2 + self.robot_w / 2):
                if self.robot.y < m.ball.y < g_hgr:
                    y = g_hgr + self.robot_w / 4
                    return (x, y)
                elif self.robot.y > m.ball.y > g_lwr:
                    y = g_lwr - self.robot_w / 4
                    return (x, y)

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

            y = (m.ball.vy / m.ball.vx) * (x - m.ball.x) + m.ball.y

            g_cvr_sup = get_cover_area(self.robot, "sup")(m.ball.x)
            g_cvr_inf = get_cover_area(self.robot, "inf")(m.ball.x)

            # trava o goleiro na lateral do gol caso a bola esteja no escanteio ou
            # acima/abaixo da linha do gol e indo para o escanteio
            if (m.ball.y > g_cvr_sup and m.ball.y > ga_hgr) or (m.ball.y > ga_hgr):
                y = g_hgr
                return (x, y)

            elif (m.ball.y < g_cvr_inf and m.ball.y < ga_lwr) or (m.ball.y < ga_lwr):
                y = g_lwr
                return (x, y)

            # bloqueia uma possivel trajetoria da bola se ela esta no meio do campo indo para a lateral
            if ga_lwr < m.ball.y < ga_hgr:
                y = m.ball.y + m.ball.vy * (16 / self.fps)

                if y > g_hgr:
                    y = g_hgr
                elif y < g_lwr:
                    y = g_lwr

                return (x, y)

            mid_field_h = self.field_h / 2

            # caso a bola esteja entre o escanteio e o meio do campo indo para uma dos escanteios,
            # o robo defende a trajetoria entre a bola e o gol
            if g_cvr_inf < m.ball.y < ga_lwr and y > self.robot.y:
                y = ((m.ball.y - mid_field_h + 0.1) / m.ball.x) * (x + self.robot_w / 2) + mid_field_h - 0.1
                if y < g_lwr:
                    return (x, g_lwr)
                else:
                    return (x, y)

            elif g_cvr_sup > m.ball.y > ga_hgr and y < self.robot.y:
                y = ((m.ball.y - mid_field_h - 0.1) / m.ball.x) * (x + self.robot_w / 2) + mid_field_h + 0.1
                if y > g_hgr:
                    return (x, g_hgr)
                else:
                    return (x, y)
            return (x, y)

        self.pebas = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|PebasBehaviour"
        )

        self.left_edge = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|LeftEdgeBehaviour"
        )

        self.right_edge = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|RightEdgeBehaviour"
        )

        self.recovery = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|RecoveryBehaviour"
        )

        self.push_ball = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|PushBallBehaviour"
        )

        self.rest = algorithms.fields.PotentialField(
            self.match,
            name=f"{self.__class__}|RestBehaviour"
        )

        self.repel = algorithms.fields.LineField(
                self.match,
                target = (-.025, 0.65),
                theta = math.pi / 2,
                line_size = 1.3,
                line_dist = 0.07,
                line_dist_max = 0.07,
                decay = lambda x: -1,
                multiplier = 1
            )

        self.get_pos = get_def_spot

        self.pebas.add_field(
            algorithms.fields.LineField(
                self.match,
                target = get_def_spot,
                theta = 0,
                line_size = 1,
                line_dist = 0.1,
                multiplier = .7,
                decay = lambda x : x**4
            )
        )

        self.left_edge.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.gk_x, self.g_left),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = 0.7
            )
        )

        self.right_edge.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.gk_x, self.g_right),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = 0.7
            )
        )

        self.push_ball.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (self.gk_x, m.ball.y),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = 0.7
            )
        )

        self.recovery.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.gk_x, .65),
                radius = 0.15,
                decay = lambda x: x**3,
                multiplier = 0.8
            )
        )

        self.rest.add_field(
            algorithms.fields.PointField(
                self.match,
                target = (self.gk_x, .65),
                radius = 0.1,
                decay = lambda x: x**2,
                multiplier = 0.5
            )
        )

        self.pebas.add_field(self.repel)
        self.recovery.add_field(self.repel)
        self.right_edge.add_field(self.repel)
        self.left_edge.add_field(self.repel)
        self.rest.add_field(self.repel)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self):

        behaviour = None

        ball = self.match.ball

        if not self.g_right < ball.y < self.g_left and ball.x < .15:
            behaviour = self.push_ball
        # elif ball.y > self.g_left:
        #     behaviour = self.left_edge
        # elif ball.y < self.g_right:
        #     behaviour = self.right_edge
        else:
            behaviour = self.pebas

        if self.robot.x > self.gk_x+0.05:
            behaviour = self.recovery

        # print(self.robot.y)

        # print(self.robot.is_visible())
        print(behaviour.name)
        return behaviour.compute([self.robot.x, self.robot.y])

    # def update(self):
    #     if self.g_right < self.robot.y < self.g_left and self.robot.x < .15 and (not -1.61 < self.robot.theta < -1.57 or not 1.57 < self.robot.theta < 1.61):
    #         w = abs(self.robot.theta) - 1.57
    #         return 0, 10*w
    #     return self.controller.update()
