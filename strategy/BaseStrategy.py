from abc import ABC

import controller

class Strategy(ABC):
    def __init__(self, match, controller=controller.SimpleLQR):
        self.match = match
        self._controller = controller

    def start(self, robot=None):
        '''
        Inicializa a estrategia
        '''
        self.controller = self._controller(robot)

        if robot:
            self.robot = robot
    
    def reset(self):
        pass

    def update(self):
        return self.controller.update()

    def set_desired(self, desired):
        self.controller.set_desired(desired)

    def decide(self):
        '''
        Calcula o proximo comando para o robo
        retorna: vetor (x, y) objetivo
        '''
        
        return self.robot.x, self.robot.y
