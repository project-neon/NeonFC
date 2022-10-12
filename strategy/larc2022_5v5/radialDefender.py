from strategy.BaseStrategy import Strategy
from algorithms.potential_fields.fields import PotentialField
import math
import numpy as np
from controller.PID_control import PID_control

class RadialDefender(Strategy):
    a = 0.3 # metade do tamanho x da elipse
    b = 0.5 # metade do tamanho y da elipse
    dist_robos = 0.000 # algo como o angulo em radianos entre o robo e o ponto central
    ponto_gol = [0,0.65] # ponto do gol (x,y)
    ponto_gol_3V3 = [0,0.65]
    ponto_g_5v5 = (0,0.9)
    ponto_objetivo = [0,0] # o melhor ponto pertencente a elipse para defender
    robo_cima = True
    mr = 0
    y_inicio_gol = 0.7
    y_final_gol = 1.1

    def __init__(self, match, name):
        controller_args = {
            'max_speed':2,
            'max_angular':9000, 
            'krho':10, 
            'kp':200,
            'ki': 0,
            'kd': 0,
        }
        super().__init__(match, name, PID_control, controller_args)

    def start(self, robot=None):
        super().start(robot=robot)
        if self.match.game.field.get_dimensions()[0] == 2.2:
            self.ponto_gol = [0,0.9]
            self.b = 0.4
            self.a = 0.4
            self.dist_robos = 0.03
 
    
    def decide(self):
        m = 0 # tangente do angulo entre a bola, o gol e o eixo x.
        mo = 0 #tangente do angulo que o robo tem que ir
        dm = 0 #tangente do angulo menorzinho que ele vai seguir
        if self.ponto_gol[0] - self.match.ball.x != 0:
            m = (self.ponto_gol[1] - self.match.ball.y)/(self.ponto_gol[0] - self.match.ball.x)
        else:
            m = (self.ponto_gol[1] - self.match.ball.y)/(0.000001)
        if self.ponto_gol[0] - self.robot.x != 0:
            self.mr = (self.ponto_gol[1] - self.robot.y)/(self.ponto_gol[0] - self.robot.x)
        else:
            self.mr = (self.ponto_gol[1] - self.robot.y)/(0.000001)
        self.calcular_ponto_gol()
        px = self.ponto_gol[0]
        py = self.ponto_gol[1]
        b = self.b
        a = self.a
        dd = 0.3 #distancia angular do robo e o ponto próximo
        erro = 0.1
        
        self.calcular_robo_cima()
        
        if self.robo_cima: # fazer os robos ficarem distanciados por um angulo.
           # m = (self.ponto_gol[1] - self.robo_baixo.y)/(self.ponto_gol[0] - self.robo_baixo.x)
           # m = (m - math.tan(self.dist_robos))/(1 + m*math.tan(self.dist_robos)) 
            # mo = np.tan(np.arctan(m) + self.dist_robos)
            if self.mr < mo + erro:
                mo = np.tan(np.arctan(m) + self.dist_robos)
            elif self.mr > mo - erro: 
                mo = (self.ponto_gol[1] - self.pos_robo_baixo[1])/(self.ponto_gol[0] - self.pos_robo_baixo[0])
                mo = np.tan(np.arctan(m) + 2*self.dist_robos)
            if self.mr < mo + erro:
                mo = np.tan(np.arctan(m) + self.dist_robos)
                dm = np.tan(np.arctan(self.mr) + dd)
            elif self.mr > mo - erro: 
                mo = (self.ponto_gol[1] - self.pos_robo_baixo[1])/(self.ponto_gol[0] - self.pos_robo_baixo[0])
                mo = np.tan(np.arctan(m) + 2*self.dist_robos)
                dm = np.tan(np.arctan(self.mr) - dd)
        else:
            #m = (self.ponto_gol[1] - self.pos_robo_cima[1])/(self.ponto_gol[0] - self.pos_robo_cima[0])
            #m = (m - math.tan(2*self.dist_robos))/(1 + m*math.tan(2*self.dist_robos))
            if self.mr < mo + erro:
                mo = (self.ponto_gol[1] - self.pos_robo_cima[1])/(self.ponto_gol[0] - self.pos_robo_cima[0])
                mo = np.tan(np.arctan(m) - 2*self.dist_robos)
            elif self.mr > mo - erro:
                mo = np.tan(np.arctan(m) - self.dist_robos)
            if self.mr < mo + erro:
                mo = (self.ponto_gol[1] - self.pos_robo_cima[1])/(self.ponto_gol[0] - self.pos_robo_cima[0])
                mo = np.tan(np.arctan(m) - 2*self.dist_robos)
                dm = np.tan(np.arctan(self.mr) + dd)
            elif self.mr > mo - erro:
                mo = np.tan(np.arctan(m) - self.dist_robos)
                dm = np.tan(np.arctan(self.mr) - dd)
        self.ponto_objetivo[0] = (2*px*dm**2/b**2 + 2*px/a**2 + math.sqrt((-2*px*dm**2/b**2 - 2*px/a**2)**2 - 4*(dm**2/b**2 + 1/a**2)*((dm*px/b)**2 - 1 + px**2/a**2)))/(2*(dm**2/b**2 + 1/a**2))
        self.ponto_objetivo[1] = dm*(self.ponto_objetivo[0] - px) + py
        self.formar_barreira(m)
        # print(np.arctan(b**2*(px - self.ponto_objetivo[0])/(a**2*(self.ponto_objetivo[1] - py))))
        #return super().decide(self.seek)
        #condição para tirar os robos da elipse caso a bola entre na pequena área
        if self.match.ball.y > 0.7 and self.match.ball.y<1.1 and self.match.ball.x<0.3:
            if self.robo_cima:
                self.ponto_objetivo=[0.41,1.45]
            else:
                self.ponto_objetivo=[0.41,0.35]
        return self.ponto_objetivo
        # return self.aim.compute([self.robot.x, self.robot.y])<<<
    
    def formar_barreira(self,m):
        limite = 1.4
        if self.robo_cima:
            if np.arctan(m) > limite:
              self.ponto_objetivo[0] = 0.00
              self.ponto_objetivo[1] = 1.3
            elif np.arctan(m) < -limite:
              self.ponto_objetivo[0] = 0.00
              self.ponto_objetivo[1] = 0.5
        else:
            if np.arctan(m) > limite:
              self.ponto_objetivo[0] = 0
              self.ponto_objetivo[1] = 1.3
            elif np.arctan(m) < -limite:
              self.ponto_objetivo[0] = 0
              self.ponto_objetivo[1] = 0.5


    def calcular_robo_cima(self):
        for robot in self.match.robots:
            
            #print(self.name)
            if "Defender" in robot.strategy.name:

                if robot.get_name() != self.robot.get_name():
                    self.pos_robo_cima = [robot.x,robot.y]
                    self.pos_robo_baixo = [robot.x,robot.y]
                    if robot.strategy.mr <= self.mr:
                        self.robo_cima = True
                    else:
                        self.robo_cima = False

    def calcular_ponto_gol(self):
        velocidade_minima = 0.1
        if self.match.ball.vx != 0 and math.sqrt(self.match.ball.vx**2 + self.match.ball.vy**2) > velocidade_minima:
            if (self.match.ball.vy/self.match.ball.vx)*(0-self.match.ball.x) + self.match.ball.y > self.y_inicio_gol and (self.match.ball.vy/self.match.ball.vx)*(0-self.match.ball.x) + self.match.ball.y < self.y_final_gol:
                self.ponto_gol[1] = (self.match.ball.vy/self.match.ball.vx)*(0-self.match.ball.x) + self.match.ball.y
            else:
                self.ponto_gol[0] = self.ponto_g_5v5[0] #alterar para 3v3 e 5v5
                self.ponto_gol[1] = self.ponto_g_5v5[1]
        else:
            self.ponto_gol[0] = self.ponto_g_5v5[0] #alterar para 3v3 e 5v5
            self.ponto_gol[1] = self.ponto_g_5v5[1]
        #print(self.ponto_gol, self.ponto_g_5v5, math.sqrt(self.match.ball.vx**2 + self.match.ball.vy**2))
        #print(self.ponto_gol, math.sqrt(self.match.ball.vx**2 + self.match.ball.vy**2))

    def update(self):
        ang = 0
        if self.ponto_objetivo[1] - self.ponto_gol[1] != 0:
            np.arctan(self.b**2*(self.ponto_gol[0] - self.ponto_objetivo[0])/(self.a**2*(self.ponto_objetivo[1] - self.ponto_gol[1])))
        else:
            np.arctan(self.b**2*(self.ponto_gol[0] - self.ponto_objetivo[0])/(self.a**2*(0.00001)))
        erro = 3.14/2
        angular_speed = 5*(abs(ang - self.robot.theta))
        
        #if math.sqrt(self.robot.vx**2 + self.robot.vy**2) >
        # if self.robot.theta < 0 and ang < 0:
        #     if self.robot.theta < ang + erro:
        #         print(f"########### GIRANDO {self.robot.get_name()}")
        #         return -angular_speed, angular_speed
        #     elif self.robot.theta >= ang - erro:
        #         print(f"########### GIRANDO {self.robot.get_name()}")
        #         return angular_speed, -angular_speed
        # elif self.robot.theta > 0 and ang > 0:
        #     if self.robot.theta > ang + erro:
        #         print(f"########### GIRANDO {self.robot.get_name()}")
        #         return angular_speed, -angular_speed
        #     elif self.robot.theta <= ang - erro:
        #         print(f"########### GIRANDO {self.robot.get_name()}")
        #         return -angular_speed, angular_speed


        # print(f"########### NORRMAL {self.robot.get_name()}")
        return self.controller.update()
        #return (   self.speed*((self.ponto[0]-self.robot.x)/self.dist),    self.speed*((self.ponto[1]-self.robot.y)/self.dist))