from univector_field import UnivectorField
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json
import math
from time import sleep

fig, ax = plt.subplots()


def initial_draw():
    artists = []
    artists.append(ax.add_patch(plt.Rectangle((0, 0), 1.5, 1.3, color='black', fill=False)))
    artists.append(ax.add_patch(plt.Rectangle((1.350, 0.300), 0.150, 0.700, color='black', fill=False)))
    artists.append(ax.add_patch(plt.Rectangle((0, 0.300), 0.150, 0.700, color='black', fill=False)))
    return artists


def draw(frame):
    uvf = UnivectorField.from_file('../../uvf_plot.json')

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

    artists = [ax.quiver(xs, ys, us, vs, color='#A23BEC'), *initial_draw()]
    for obstacle in uvf.obstacles:
        artists.append(ax.add_patch(plt.Rectangle((obstacle.center[0] - 0.075*0.5, obstacle.center[1] - 0.075*0.5),
                                                  0.075,
                                                  0.075,
                                                  color='blue',
                                                  fill=True)))
    artists.append(ax.add_patch(plt.Circle(uvf.g, 21.5e-3, color="orange", fill=True)))
    artists.append(ax.add_patch(plt.Circle(uvf.r, 1.e-3, color="red", fill=True)))
    return artists

#for i in initial_draw():
#    ax.add_patch(i)
ani = animation.FuncAnimation(fig, draw, init_func=initial_draw,interval=10, blit=True)
plt.show()
