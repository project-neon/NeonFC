from abc import ABC, abstractmethod
from entities.visualization import logger
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Visualizer(ABC):
    def __init__(self):
        self.readers = {table: logger.Reader(table) for table in self.tables}
        self.values = {table: None for table in self.tables}
        self.__animation_object = None

    def update(self):
        self.values = {table: reader.read() for table, reader in self.readers.items()}


class MPLVisualizer(Visualizer):
    def __init__(self):
        super().__init__()

        self.fig, self.ax = plt.subplots()

    @abstractmethod
    def _draw(self, frame, **kwargs):
        pass

    def animation(self, interval=1):
        self.__animation_object = animation.FuncAnimation(self.fig, self._draw, interval=interval, blit=True)
        return self.__animation_object

    def __call__(self, interval=10):
        return self.animation(interval=interval)
