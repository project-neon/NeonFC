import math


def get_closest_ellipse_position(ballPos, origin, radiusX, radiusY):
    ratio = radiusX / radiusY
    vec = {
        'x': (ballPos['x'] - origin['x']),
        'y': (ballPos['y'] - origin['y']) * ratio
    }
    ang = math.atan2(vec['x'], vec['y'])
    vec = {
        'x': origin['x'] + math.sin(ang) * radiusX,
        'y': origin['y'] + math.cos(ang) * radiusY,
        'ang_clockwise': (ang + math.pi / 2) % math.pi,
        'ang_counter_clockwise': (ang - math.pi / 2) % math.pi
    }
    return vec
