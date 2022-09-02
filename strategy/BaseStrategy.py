from abc import ABC

import controller
from controller.uni_controller import UniController

class Strategy(ABC):
    def __init__(self, match, name, controller=UniController, controller_kwargs={}):
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

    def set_desired(self, desired, desired_dl=None):
        if self.controller.__class__ is UniController:
            self.controller.set_desired(self.match, desired, desired_dl)
        else:
            self.controller.set_desired(desired)

    def decide(self):
        '''
        Calcula o proximo comando para o robo
        retorna: vetor (x, y) objetivo
        '''
        
        return self.robot.x, self.robot.y
