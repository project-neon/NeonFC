from strategy.BaseStrategy import Strategy
from algorithms.potential_fields.fields import PotentialField, PointField, LineField, TangentialField
import math
import numpy as np
from controller.simple_LQR import TwoSidesLQR
from controller.PID_control import PID_control

from strategy.DebugTools import DebugPotentialFieldStrategy
import matplotlib.pyplot as plt

#class Attacker(DebugPotentialFieldStrategy):
class Attacker(Strategy):
    a = 0.3 # metade do tamanho x da elipse
    b = 0.5 # metade do tamanho y da elipse
    dist_robos = 3.14/30 # algo como o angulo em radianos entre o robo e o ponto central
    ponto_gol = [0,0.65] # ponto do gol (x,y)
    ponto_objetivo = [0,0] # o melhor ponto pertencente a elipse para defender
    robo_cima = True
    mr = 0

    def __init__(self, match, name):
        controller_args = {}
        super().__init__(match, name, PID_control)

    def start(self, robot=None):


        super().start(robot=robot)
        
        if self.match.game.field.get_dimensions()[0] == 2.2:
            self.ponto_gol = [0,0.9]
            self.b = 0.6
            self.a = 0.4
            self.dist_robos = 3.14/60
        self.seek = PotentialField(self.match,name="SeekBehaviour")

        self.aim = PotentialField(self.match,name="AimBehaviour")

        self.carry = PotentialField(self.match,name="CarryBehaviour")

           
            
    
    def decide(self):
        m = (self.ponto_gol[1] - self.match.ball.y)/(self.ponto_gol[0] - self.match.ball.x) # tangente do angulo entre a bola, o gol e o eixo x.
        self.mr = (self.ponto_gol[1] - self.robot.y)/(self.ponto_gol[0] - self.robot.x)
        px = self.ponto_gol[0]
        py = self.ponto_gol[1]
        b = self.b
        a = self.a
        dd = 0.2 #distancia angular do robo e o ponto próximo

        for robot in self.match.robots:
            
            #print(self.name)
            if "defender" in robot.strategy.name:

                if robot.get_name() != self.robot.get_name():
                    if robot.strategy.mr <= self.mr:
                        self.robo_cima = True
                    else:
                        self.robo_cima = False
            

        
        if self.robo_cima: # fazer os robos ficarem distanciados por um angulo.
            m = (m + math.tan(self.dist_robos))/(1 - m*math.tan(self.dist_robos)) 
        else:
            m = (m - math.tan(self.dist_robos))/(1 + m*math.tan(self.dist_robos))
        if self.mr < m - 0.2:
            m = (self.mr + math.tan(dd))/(1 - self.mr*math.tan(dd))
        elif self.mr > m + 0.2: 
            m = (self.mr - math.tan(dd))/(1 + self.mr*math.tan(dd))
        self.ponto_objetivo[0] = (2*px*m**2/b**2 + 2*px/a**2 + math.sqrt((-2*px*m**2/b**2 - 2*px/a**2)**2 - 4*(m**2/b**2 + 1/a**2)*((m*px/b)**2 - 1 + px**2/a**2)))/(2*(m**2/b**2 + 1/a**2))
        
        self.ponto_objetivo[1] = m*(self.ponto_objetivo[0] - px) + py
        
        #return super().decide(self.seek)
        return self.ponto_objetivo
        # return self.aim.compute([self.robot.x, self.robot.y])
        
        #return (   self.speed*((self.ponto[0]-self.robot.x)/self.dist),    self.speed*((self.ponto[1]-self.robot.y)/self.dist))