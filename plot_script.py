import sys
import random

from itertools import count
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

x_vals = deque(maxlen=200)
y_vals = deque(maxlen=200)

index = count()

def animate(i):
    data = sys.stdin.readline()
    parts = [float(y) for y in data.split(',')]
    x_vals.append(next(index))
    y_vals.append(parts)

    plt.cla()
    plt.plot(x_vals, y_vals)

ani = FuncAnimation(plt.gcf(), animate, interval=1)

plt.tight_layout()
plt.show()