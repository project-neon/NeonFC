import math

import numpy as np

class DiscreteField(object):
    def __init__(self, geometry={}, resolution=50):
        '''
        resolution: area em centimetro da discretização do espaço.
        '''
        super().__init__()

        self.resolution = resolution
        self.field_width = 2000 # geometry.get('width') # cm
        self.field_height = 1800 # geometry.get('height') # cm

        self.create_matrix()

    def create_matrix(self):
        self.cells_w = math.floor(self.field_width/self.resolution)
        self.cells_h = math.floor(self.field_height/self.resolution)

        # cria uma matriz discretizada do campo de tamanho matrix[width][height] -> x,y
        self.matrix = [
            [0 for w in range(int(self.cells_h)) ] for i in range(int(self.cells_w))
        ]
    
    def clean_matrix(self):
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                self.matrix[i][j] = 0

    def _fm(self, x, y):
        '''
        field geometry -> discrete geometry
        recebe: (x, y) da geometria do jogo, 
        retorna: (x, y) na matrix discreta docampo
        ''' 
        return math.floor(x * 1000/self.resolution), math.floor(y * 1000/self.resolution)

    def update(self, avoiances=[], target=(0, 0), position=(0, 0)):
        '''
        recebe:
            avoiances -> {x: cm, y: cm, radius: cm}
            target -> (x, y)
            position -> (x, y)
        '''
        self.clean_matrix()
        for obstacle in avoiances:
            x, y, radius = obstacle['x'], obstacle['y'], obstacle.get('radius', 0)
            self._paint_forbidden(x, y, radius)

    def _paint_forbidden(self, x, y, radius):
        x, y = self._fm(x, y)
        radius = int(radius/self.resolution)
        # TODO caso o robo não esteja dentro do grid que representa
        # o campo, talvez seja bom logar uma mensagem de erro
        if x < self.cells_w and y < self.cells_h:
            self.matrix[x][y] = 1

            for i in range(x - radius, x + radius):
                for j in range(y - radius, y + radius):
                    # caso onde sai do campo
                    if x < 0 or y < 0 or x >= self.cells_h or y >= self.cells_w:
                        continue

                    # fora do raio
                    if math.sqrt( math.pow((x - i), 2) + math.pow((y - j), 2)) <= radius:
                        self.matrix[i][j] = 1


if __name__ == "__main__":
    field = DiscreteField(resolution=5)

    field.update(
        avoiances=[
            {'x': 1000, 'y': 1000, 'radius': 75}
        ]
    )
