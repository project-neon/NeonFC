import sys
import random
import subprocess

from itertools import count
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

y_vals_a = []
y_vals_l = []

fig, axs = plt.subplots(2)

fig.suptitle('PID Controller')

with open("pid.log", "r") as a_file:
    for line in a_file:
        parts_l = [float(y) for y in line.split(',')][:2]
        parts_a = [float(y) for y in line.split(',')][3:-1]

        y_vals_l.append(parts_l)
        y_vals_a.append(parts_a)


line_0bjects = axs[0].plot(y_vals_l)
axs[0].legend(iter(line_0bjects), ('Desired', 'Actual'))

line_0bjects = axs[1].plot(y_vals_a)
axs[1].legend(iter(line_0bjects), ('Desired', 'Actual'))

plt.tight_layout()
plt.show()