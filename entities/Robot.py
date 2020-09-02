import math
import numpy as np

from collections import deque

from scipy.ndimage.interpolation import rotate

def speed(_list, _fps):
    if len(_list) <= 1:
        return 0
    
    speed_fbf = [
        (v - i) for i, v 
        in zip(
            _list, 
            list(_list)[1:]
        )
    ]

    return _fps * (sum(speed_fbf)/len(speed_fbf))

def unit_vector(vector):
    """ Returns the unit vector of the vector."""
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """Finds angle between two vectors"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def rotate_via_numpy(xy, radians):
    """Use numpy to build a rotation matrix and take the dot product."""
    x, y = xy
    c, s = np.cos(radians), np.sin(radians)
    j = np.matrix([[c, s], [-s, c]])
    m = np.dot(j, [x, y])

    return float(m.T[0]), float(m.T[1])


class Robot(object):

    def __init__(self, game, robot_id, team_color):
        self.game = game
        self.robot_id = robot_id
        self.team_color = team_color
        self.current_data = {}

        self._frames = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10),
            'theta': deque(maxlen=10)
        }

        self.vx, self.vy, self.vtheta = 0, 0, 0
    
    def get_name(self):
        return 'ROBOT_{}_{}'.format(self.robot_id, self.team_color)

    def update(self, frame):
        team_color_key = 'robotsBlue' if self.team_color == 'blue' else 'robotsYellow'

        robot_data = [i for i in frame[team_color_key] if i.get('robotId') == self.robot_id]

        if len(robot_data) >= 1:
            self.current_data = robot_data[0]
        else:
            print('## CAUTION: robo não encontrado')
            raise ValueError('## CAUTION: robo não encontrado')
            
        
        self._update_speeds()

    def _update_speeds(self):
        self._frames['x'].append(self.current_data['x'])
        self._frames['y'].append(self.current_data['y'])
        self._frames['theta'].append(self.current_data['orientation'])

    
        self.theta = self.current_data['orientation']
        self.vx = speed(self._frames['x'], self.game.vision._fps)
        self.vy = speed(self._frames['y'], self.game.vision._fps)
        self.vtheta = speed(self._frames['theta'], self.game.vision._fps)

        # print(self.get_name(), '= speeds: vx: {:.4f} m/s :: vy: {:.4f} m/s :: vt: {:.2f} RAD/s'.format(self.vx, self.vy, self.vtheta))


    def _get_differential_robot_speeds(self, vx, vy, theta):
        '''
        Entradas: velocidades no eixo X, Y
        Saidas: velocidades linear, angular
        '''
        speed_vector = np.array([vx, vy])

        speed_norm = np.linalg.norm(speed_vector)
        robot_world_speed = rotate_via_numpy(speed_vector, theta)

        vl = robot_world_speed[0] * speed_norm

        va = -angle_between(robot_world_speed, (1, 0)) * speed_norm

        return vl, va
        

    def decide(self):
        # mocado, for a while :)
        power_left, power_right = 80, 50

        self._get_differential_robot_speeds(self.vx, self.vy, self.theta)

        return self._get_command(power_left, power_right)


    def _get_command(self, pl, pr):
        return {
            'robot_id': self.robot_id,
            'wheel_left': pl,
            'wheel_right': pr,
            'color': self.team_color
        }
