from abc import ABC

import controller

class Strategy(ABC):
    def __init__(self, match, name, controller=controller.SimpleLQR, controller_kwargs={}):
        self.match = match
        self._controller = controller
        self._ctr_kwargs = controller_kwargs
        self.name = name

    def start(self, robot=None):
        '''
        Inicializa a estrategia
        '''
        self.controller = self._controller(robot, **self._ctr_kwargs)

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