import math
import logging
import numpy as np

from collections import deque
from scipy.ndimage.interpolation import rotate

import controller
import algorithims


from commons.math import angular_speed, speed, rotate_via_numpy, unit_vector

class Robot(object):

    def __init__(self, game, robot_id, team_color):
        self.game = game
        self.robot_id = robot_id
        self.team_color = team_color
        self.current_data = {}

        self.astar = algorithims.AStar()


        self.log = logging.getLogger(self.get_name())
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)

        formatter = logging.Formatter('\033[1m|%(levelname)s|%(name)s|%(message)s\033[1m')
        ch.setFormatter(formatter)
        self.log.addHandler(ch)

        self.dimensions = {
            'L': 0.075,
            'R': 0.02
        }

        self.controller = controller.SimpleLQR(self)
        self.power_left, self.power_right = 0, 0

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
            self.log.warn('Robo [{}] não encontrado, pode estar desligado!'.format(self.get_name()))
            return

        self._update_speeds()

    def _update_speeds(self):
        self._frames['x'].append(self.current_data['x'])
        self._frames['y'].append(self.current_data['y'])
        self._frames['theta'].append(self.current_data['orientation'])

    
        self.theta = self.current_data['orientation']

        self.x = self.current_data['x']
        self.y = self.current_data['y']

        self.vx = speed(self._frames['x'], self.game.vision._fps)
        self.vy = speed(self._frames['y'], self.game.vision._fps)
        self.vtheta = angular_speed(self._frames['theta'], self.game.vision._fps)

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

        va = self.vtheta
        
        return vl, va
        

    def decide(self):
        self.astar.update_field(
            obstacles=[
                {
                    "x": r.x, 
                    "y": r.y
                } for r in self.game.match.opposites + self.game.match.robots if not (r.team_color == self.team_color and r.robot_id == self.robot_id)
            ]
        )

        self.astar.calculate_when(
            (self.x, self.y),
            (self.game.match.ball.x, self.game.match.ball.y),
            timespan = 10
        )

        objective = self.astar.next_node(self.x, self.y)
        desired = unit_vector([(objective[0] - self.x), (objective[1] - self.y)]) * 3000

        self.controller.set_desired(desired)

        self.power_left, self.power_right = self.controller.update()
       
        return self._get_command(self.power_left, self.power_right)


    def _get_command(self, pl, pr):
        return {
            'robot_id': self.robot_id,
            'wheel_left': pl,
            'wheel_right': pr,
            'color': self.team_color
        }
