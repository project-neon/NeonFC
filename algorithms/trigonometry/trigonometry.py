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


def get_closest_ellipse_position_pure(ballPosX, ballPosY, originX, originY, radiusX, radiusY):
    ratio = radiusX / radiusY
    ang = math.atan2(ballPosX - originX, (ballPosY - originY) * ratio)
    return originX + math.sin(ang) * radiusX, originY + math.cos(ang) * radiusY, (ang + math.pi / 2) % math.pi, (ang - math.pi / 2) % math.pi
