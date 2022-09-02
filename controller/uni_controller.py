import math

"""
Controle baseado em angulo desejado
Referente ao soccer robotics
"""

class UniController(object):
    def __init__(self, robot):
        self.robot = robot
        self.L = self.robot.dimensions.get("L") #m
        self.R = self.robot.dimensions.get("R") #m
        self.V_M = 149 #m/s
        self.R_M = 3*self.V_M #rad*m/s
        self.K_W = 7 #coeficiente de feedback #20
        self.K_P = 1
        self.v1 = 0 #restricao de velocidade 1
        self.v2 = 0 #restricao de velocidade 2
        self.theta_d = 0
        self.theta_f = 0
        self.dl = 0.000001 #aproximar phi_v em m
        self.phi_v = 0
        self.a_phi_v = 0 #absoluto de phi_v
        self.theta_e = 0
        self.a_theta_e = 0 #absoluto de theta_e


    def control(self):
        """
        x, y, q: postura do robo (posicao x, posicao t, angulo do robo)
        retorna: v e w, velocidade linear e velocidade angular respectivamente
        """
        self.phi_v = self.theta_f - self.theta_d

        while self.phi_v > math.pi:
            self.phi_v -= 2*math.pi
        while self.phi_v < -math.pi:
            self.phi_v += 2*math.pi
        
        self.phi_v = self.phi_v/self.dl
        self.a_phi_v = abs(self.phi_v)

        #calculate theta_e
        self.theta_e = self.theta_d - self.robot.theta

        while self.theta_e > math.pi:
            self.theta_e -= 2*math.pi
        while self.theta_e < -math.pi:
            self.theta_e += 2*math.pi
        
        self.a_theta_e = abs(self.theta_e)

        #calculate v
        self.v1 = (
            (2*self.V_M - self.L*self.K_W*math.sqrt(self.a_theta_e))/
            (2 + self.L*self.a_phi_v)
        )

        self.v2 = (
            (math.sqrt(
                (self.K_W**2) + 4*self.R_M*self.a_phi_v
                ) - self.K_W * math.sqrt(self.a_theta_e)
            ) / (2 * self.a_phi_v + self.dl)
        )

        ball_x, ball_y = self.match.ball.x, self.match.ball.y

        self.v3 = self.K_P * (self.robot.x - ball_x)
        
        v = min(self.v1, self.v2)

        #calcular w
        if self.theta_e > 0:
            w = v * self.phi_v + self.K_W * math.sqrt(self.a_theta_e)
        else:
            w = v * self.phi_v - self.K_W * math.sqrt(self.a_theta_e)

        return v, w

    def set_desired(self, match, theta_d, theta_f):
        self.match = match
        # vx = theta[0]
        # vy = theta[1]
        # theta = math.atan2(vy, vx)

        self.theta_d = theta_d
        # vx_f, vy_f = self.robot.strategy.decide(
        #     self.robot.x + self.dl * math.cos(self.robot.theta),
        #     self.robot.y + self.dl * math.sin(self.robot.theta)
        # )
        self.theta_f = theta_f

    def update(self):
        v, w = self.control()

        pwr_left = v - 0.5 * self.L * w
        pwr_right = v + 0.5 * self.L * w

        print(f"{v=}\n{w=}")

        return v, w