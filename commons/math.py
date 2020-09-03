import math
import numpy as np

def _fix_angle(theta_1, theta_2):
    rate_theta = (theta_2 - theta_1)
  
    if (rate_theta > math.pi ):
        rate_theta -= 2 * math.pi
    elif (rate_theta < -math.pi):
        rate_theta += 2 * math.pi

    return rate_theta


def angular_speed(_list, _fps):
    if len(_list) <= 1:
        return 0
    
    speed_fbf = [
        _fix_angle(t0, t1) for t0, t1 
        in zip(
            _list, 
            list(_list)[1:]
        )
    ]

    return _fps * (sum(speed_fbf)/len(speed_fbf))


def speed(_list, _fps):
    if len(_list) <= 1:
        return 0
    
    speed_fbf = [
        (t1 - t0) for t0, t1 
        in zip(
            _list, 
            list(_list)[1:]
        )
    ]

    return _fps * (sum(speed_fbf)/len(speed_fbf))


def unit_vector(vector):
    """ Returns the unit vector of the vector."""
    if np.linalg.norm(vector) == 0:
        return np.array([0, 0])
    return vector / np.linalg.norm(vector)

def rotate_via_numpy(xy, radians):
    """Use numpy to build a rotation matrix and take the dot product."""
    x, y = xy
    c, s = np.cos(radians), np.sin(radians)
    j = np.matrix([[c, s], [-s, c]])
    m = np.dot(j, [x, y])

    return float(m.T[0]), float(m.T[1])