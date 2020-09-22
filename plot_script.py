import sys
import random

from itertools import count
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

x_vals = deque(maxlen=50)
y_vals_a = deque(maxlen=50)
y_vals_l = deque(maxlen=50)

index = count()

fig, axs = plt.subplots(2)

fig.suptitle('PID Controller')

def animate(i):
    data = sys.stdin.readline()
    sys.stdout.write(data)
    sys.stdout.flush()
    parts_l = [float(y) for y in data.split(',')][:3]
    parts_a = [float(y) for y in data.split(',')][3:]

    x_vals.append(next(index))
    y_vals_l.append(parts_l)
    y_vals_a.append(parts_a)

    axs[0].cla()
    line_0bjects = axs[0].plot(x_vals, y_vals_l)
    axs[0].legend(iter(line_0bjects), ('Desired', 'Actual', 'processed'))

    axs[1].cla()
    line_0bjects = axs[1].plot(x_vals, y_vals_a)
    axs[1].legend(iter(line_0bjects), ('Desired', 'Actual', 'processed'))

ani = FuncAnimation(plt.gcf(), animate, interval=0)

plt.tight_layout()
plt.show()