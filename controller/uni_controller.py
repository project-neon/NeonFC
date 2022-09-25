import math

"""
Controle baseado em angulo desejado
Referente ao soccer robotics
"""

''' simulado

self.V_M = 200
self.R_M = 3 * self.V_M
self.K_W = 16.5
'''

class UniController(object):
    def __init__(self, robot):
        self.robot = robot
        self.L = self.robot.dimensions.get("L")  # m
        self.R = self.robot.dimensions.get("R")  # m
        self.V_M = 20 # m/s
        self.R_M = 5 * self.V_M # rad*m/s
        self.K_W = .3 # coeficiente de feedback
        self.K_P = 5
        self.v1 = 0  # restricao de velocidade 1
        self.v2 = 0  # restricao de velocidade 2
        self.theta_d = 0
        self.theta_f = 0
        self.dl = 0.000001  # aproximar phi_v em m
        self.phi_v = 0
        self.a_phi_v = 0  # absoluto de phi_v
        self.theta_e = 0
        self.a_theta_e = 0  # absoluto de theta_e

    def control(self):
        """
        x, y, q: postura do robo (posicao x, posicao t, angulo do robo)
        retorna: v e w, velocidade linear e velocidade angular respectivamente
        """

        self.phi_v = self.theta_f - self.theta_d

        while self.phi_v > math.pi:
            self.phi_v -= 2 * math.pi
        while self.phi_v < -math.pi:
            self.phi_v += 2 * math.pi

        self.phi_v = self.phi_v / self.dl
        self.a_phi_v = abs(self.phi_v)

        # calculate theta_e
        self.theta_e = self.theta_d - self.robot.theta

        while self.theta_e > math.pi:
            self.theta_e -= 2 * math.pi
        while self.theta_e < -math.pi:
            self.theta_e += 2 * math.pi

        self.a_theta_e = abs(self.theta_e)

        # calculate v
        self.v1 = (2 * self.V_M - self.L * self.K_W * math.sqrt(self.a_theta_e)) / (2 + self.L * self.a_phi_v)

        self.v2 = (math.sqrt(self.K_W ** 2 + 4 * self.R_M * self.a_phi_v) - self.K_W * math.sqrt(self.a_theta_e)) \
                  / (2 * self.a_phi_v) if self.a_phi_v > 0 else self.V_M

        ball_x, ball_y = self.match.ball.x, self.match.ball.y

        self.v3 = self.K_P * ((self.robot.x - ball_x) ** 2 + (self.robot.y - ball_y) ** 2) ** .5
        #self.v3 = self.K_P * ((self.robot.x - .75) ** 2 + (self.robot.y - .65) ** 2) ** .5
        print(f"{self.v3=}")

        v = min(self.v1, self.v2)#, self.v3)

        # calcular w
        if self.theta_e > 0:
            w = v * self.phi_v + self.K_W * math.sqrt(self.a_theta_e)
        else:
            w = v * self.phi_v - self.K_W * math.sqrt(self.a_theta_e)

        print(f"{self.robot.speed/v:.2f}, {self.robot.vtheta/w:.2f}")

        w *= -7

        return v, w

    def set_desired(self, match, theta_d, theta_f):
        self.match = match

        self.theta_d = theta_d
        self.theta_f = theta_f

    def update(self):
        v, w = self.control()

        pwr_left = v - 0.5 * self.L * w
        pwr_right = v + 0.5 * self.L * w

        # return pwr_left, pwr_right
        return v, w
