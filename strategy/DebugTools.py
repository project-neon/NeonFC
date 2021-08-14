from strategy.BaseStrategy import Strategy

from algorithms.potential_fields.plotter import PotentialDataExporter

import controller

class DebugPotentialFieldStrategy(Strategy):
    def __init__(self, match, name, controller=controller.SimpleLQR, controller_kwargs={}):
        super().__init__(match, name, controller, controller_kwargs)
        self.analyzed = None

    def start(self, robot=None):
        super().start(robot)
        self.exporter = PotentialDataExporter(self.robot.get_name())
    
    def decide(self, behaviour):
        '''
        recebe o Potential Field para que a analise seja feita e retorna 0, 0
        '''
        self.exporter.export(behaviour, self.robot, self.match.ball)

        return [0, 0]
