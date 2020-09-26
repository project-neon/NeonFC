from abc import ABC

class Strategy(ABC):
    def __init__(self, match):
        self.match = match

    def start(self, robot=None):
        '''
        Inicializa a estrategia
        '''
        if robot:
            self.robot = robot
    
    def reset(self):
        pass

    def decide(self):
        '''
        Calcula o proximo comando para o robo
        retorna: vetor (x, y) objetivo
        '''
        
        return self.robot.x, self.robot.y
