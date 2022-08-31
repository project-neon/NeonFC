import math
import numpy as np

def angle_adjustment(angle):
        phi = angle % math.radians(360)
        if phi > math.radians(180):
            phi = phi - math.radians(360)