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
        self.V_M = 1 #m/s
        self.R_M = 3 #rad*m/s
        self.K_W = 5 #coeficiente de feedback

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
            (2 * self.V_M
            - self.L * self.K_W * math.sqrt(self.a_theta_e)) /
            (2 + self.L * self.a_phi_v)
        )

        if self.a_phi_v > 0:
            self.v2 = (
                (math.sqrt(
                    (self.K_W ** 2) * self.a_theta_e +
                    4 * self.R_M * self.a_phi_v) -
                self.K_W * math.sqrt(self.a_theta_e)) /
                (2 * self.a_phi_v)
            )
        else:
            self.v2 = self.V_M
        
        v = min(self.v1, self.v2)

        #calcular w
        if self.theta_e > 0:
            w = v * self.phi_v + self.K_W * math.sqrt(self.a_theta_e)
        else:
            w = v * self.phi_v - self.K_W * math.sqrt(self.a_theta_e)

        return v, w

    def set_desired(self, theta):
        vx = theta[0]
        vy = theta[1]
        theta = math.atan2(vy, vx)

        self.theta_d = theta
        vx_f, vy_f = self.robot.strategy.decide(
            self.robot.x + self.dl * math.cos(self.robot.theta),
            self.robot.y + self.dl * math.sin(self.robot.theta)
        )
        self.theta_f = math.atan2(vy_f, vx_f)

    def update(self):
        v, w = self.control()

        pwr_left = (2 * v - w * self.L)/2 * self.R * 1000
        pwr_right = (2 * v + w * self.L)/2 * self.R * 1000

        print(pwr_left, pwr_right)

        return pwr_left, pwr_right

