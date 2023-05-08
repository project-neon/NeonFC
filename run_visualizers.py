from live_plot.visualizer import MPLVisualizer
from algorithms import UnivectorField
from collections import deque
import math
import numpy as np
import matplotlib.pyplot as plt
import time


def main():
    #uvf_visualizer = UVFVisualizer()
    #uvf_visualizer()

    # pid_visualizer = PIDVisualizer(500)
    # pid_visualizer()

    position_visualizer = PositionVisualizer(1000)
    position_visualizer(1)

    circuit = [(1.1, .40), (1.1, .90), (.75, .65), (.4, .90), (.4, .40)]
    x, y = list(map(list, zip(*circuit)))

    ax = plt.gca()

    # ax.add_patch(plt.Circle((.75, .65 + .08), .08, color="blue", fill=None))
    # ax.add_patch(plt.Circle((.75, .65 - .08), .08, color="blue", fill=None))

    plt.scatter(x, y, c='r')
    plt.show()

class PositionVisualizer(MPLVisualizer):
    def __init__(self, n):
        self.lt = time.time()
        self.tables = ['robot']
        super().__init__()

        self.xs = deque(maxlen=n)
        self.ys = deque(maxlen=n)

    def __update(self):
        t = time.time()
        print(1/(t-self.lt))
        self.lt=t

        self.update()

        try:
            self.xs.append(self.values['robot']['x'])
            self.ys.append(self.values['robot']['y'])
        except TypeError:
            pass


    def _draw(self, frame, **kwargs):
        self.__update()

        plt.xlim([0, 1.5])
        plt.ylim([0, 1.3])

        return [self.ax.scatter(self.xs, self.ys, s=1, c='black')]



class PIDVisualizer(MPLVisualizer):
    def __init__(self, n):
        self.tables = ['pid']
        super().__init__()

        self.set_points = deque(maxlen=n)
        self.errors = deque(maxlen=n)
        self.times = deque(maxlen=n)
        self.inputs = deque(maxlen=n)
        self.ws = deque(maxlen=n)

        self.t0 = None
        self.__update(True)

    def __update(self, set_t0=False):
        self.update()

        if set_t0:
            self.t0 = self.values['pid']['time']

        try:
            self.set_points.append(self.values['pid']['set_point'])
            self.errors.append(self.values['pid']['error'])
            self.inputs.append(self.values['pid']['set_point'] - self.values['pid']['error'])
            self.times.append(self.values['pid']['time'] - self.t0)
            self.ws.append(self.values['pid']['w']/50)
        except TypeError:
            pass

        print(self.times[-1], self.set_points[-1])


    def _draw(self, frame, **kwargs):
        self.__update()

        plt.xlim([self.times[0], self.times[-1]])

        return [self.ax.plot(self.times, self.set_points, 'r--')[0],
                self.ax.plot(self.times, self.inputs, 'black')[0]]#,
                #self.ax.plot(self.times, self.ws, 'g:')[0]]



class UVFVisualizer(MPLVisualizer):
    def __init__(self):
        self.tables = ['ball', 'robot', 'uvf']
        super().__init__()

    def _draw(self, frame, **kwargs):
        self.update()

        uvf = UnivectorField(self.values['uvf']['n'], 0)
        g = (self.values['uvf']['gx'], self.values['uvf']['gy'])
        r = (self.values['uvf']['rx'], self.values['uvf']['ry'])
        uvf.set_target(g, r)
        xs, ys = np.meshgrid(np.linspace(0, 1.5, 53), np.linspace(0, 1.3, 53))
        us, vs = [], []

        for x, y in zip(np.nditer(xs), np.nditer(ys)):
            ang = uvf((x, y))

            u = math.cos(ang)
            v = math.sin(ang)

            us.append(u)
            vs.append(v)

        us = np.array(us).reshape(xs.shape)
        vs = np.array(vs).reshape(ys.shape)

        artists = [self.ax.quiver(xs, ys, us, vs, color='#A23BEC'),
                   self.ax.add_patch(plt.Rectangle((0, 0), 1.5, 1.3, color='black', fill=False)),
                   self.ax.add_patch(plt.Rectangle((1.350, 0.300), 0.150, 0.700, color='black', fill=False)),
                   self.ax.add_patch(plt.Rectangle((0, 0.300), 0.150, 0.700, color='black', fill=False))]

        for obstacle in uvf.obstacles:
            artists.append(
                self.ax.add_patch(plt.Rectangle((obstacle.center[0] - 0.075 * 0.5, obstacle.center[1] - 0.075 * 0.5),
                                           0.075,
                                           0.075,
                                           color='blue',
                                           fill=True)))
        artists.append(self.ax.add_patch(plt.Circle(uvf.g, 21.5e-3, color="orange", fill=True)))
        artists.append(self.ax.add_patch(plt.Circle(uvf.r, 1.e-3, color="red", fill=True)))

        return artists


if __name__ == "__main__":
    main()
