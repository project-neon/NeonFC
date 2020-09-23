import sys
import random
import subprocess

from itertools import count
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

x_vals = deque(maxlen=100)
y_vals_a = deque(maxlen=100)
y_vals_l = deque(maxlen=100)

index = count()

fig, axs = plt.subplots(2)

fig.suptitle('PID Controller')

def animate(i):
    line = subprocess.check_output(['tail', '-1', 'pid.log']).decode()

    parts_l = [float(y) for y in line.split(',')][:2]
    parts_a = [float(y) for y in line.split(',')][3:-1]

    x_vals.append(next(index))
    y_vals_l.append(parts_l)
    y_vals_a.append(parts_a)

    axs[0].cla()
    line_0bjects = axs[0].plot(x_vals, y_vals_l)
    axs[0].legend(iter(line_0bjects), ('Desired', 'Actual'))

    axs[1].cla()
    line_0bjects = axs[1].plot(x_vals, y_vals_a)
    axs[1].legend(iter(line_0bjects), ('Desired', 'Actual'))

ani = FuncAnimation(plt.gcf(), animate, interval=0)

plt.tight_layout()
plt.show()