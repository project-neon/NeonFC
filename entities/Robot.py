import math
import logging
import numpy as np

from collections import deque
from scipy.ndimage.interpolation import rotate

import controller
import algorithims

import strategy

from commons.math import angular_speed, speed, rotate_via_numpy, unit_vector

class Robot(object):

    def __init__(self, game, robot_id, team_color):
        self.game = game
        self.robot_id = robot_id
        self.team_color = team_color
        self.current_data = {}

        if self.robot_id == 0:
            self.strategy = strategy.tests.GoalKeeper(game.match)
        elif self.robot_id == 1:
            self.strategy = strategy.tests.FollowBall(game.match)

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

        self.controller = controller.Robot_PID(self)
        self.power_left, self.power_right = 0, 0

        self._frames = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10),
            'theta': deque(maxlen=10)
        }

        self.vx, self.vy, self.vtheta = 0, 0, 0

        self.strategy.start(self)
    
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


    def _get_desired_differential_robot_speeds(self, vx, vy, theta):
        '''
        Entradas: velocidades no eixo X, Y
        Saidas: velocidades linear, angular
        '''
    
        speed_vector = np.array([vx, vy])
        speed_norm = np.linalg.norm(speed_vector)
        robot_world_speed = list(rotate_via_numpy(speed_vector, theta))
        vl = robot_world_speed[0] * speed_norm

        # # code to make the robot move to both directions
        # if robot_world_speed[0] > 0.0:
        #     robot_world_speed[1] = -robot_world_speed[1]
        #     robot_world_speed[0] = -robot_world_speed[0]
        
        def _map(min_i, max_i, min_o, max_o, x):
            return (x - min_i) * (max_o - min_o) / (max_i - min_i) + min_o
        
        robot_angle_speed = -math.atan2(robot_world_speed[1], robot_world_speed[0])

        damping = 1
        if (abs(robot_world_speed[1]) + abs(robot_world_speed[0])):
            damping = max(1, abs((robot_world_speed[1])/(abs(robot_world_speed[1]) + abs(robot_world_speed[0])))) * min(1, max(0, _map(0.01, 0.05, 0, 1, speed_norm)))
        
        # TODO Discover magic number after PID testing
        va = robot_angle_speed * damping
        return vl, va


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
        desired = self.strategy.decide()
        # desired = unit_vector( [(self.game.match.ball.x - self.x), (self.game.match.ball.y - self.y)]) / 2

        self.controller.set_desired(desired)

        self.power_left, self.power_right = self.controller.update()
        if self.robot_id == 0:
            # print("--------------", self.power_left, self.power_right)
            # print('::::::::::::::::::::', self._get_differential_robot_speeds(self.vx, self.vy, self.theta))
            pass
        return self._get_command(self.power_left, self.power_right)


    def _get_command(self, pl, pr):
        return {
            'robot_id': self.robot_id,
            'wheel_left': pl,
            'wheel_right': pr,
            'color': self.team_color
        }
