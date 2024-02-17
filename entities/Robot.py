import math
import logging
import numpy as np
from collections import deque
from commons.math import angular_speed, rotate_via_numpy
from commons.math import speed as avg_speed


class Robot(object):

    def __init__(self, game, robot_id, team_color):
        self.game = game
        self.robot_id = robot_id
        self.team_color = team_color
        self.current_data = {}

        self.strategy = None

        self.stuck_time = 0

        """
        Essas atribuições serão feitas no Coach quando ele existir
        """

        self.log = logging.getLogger(self.get_name())
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)

        formatter = logging.Formatter('\033[1m|%(levelname)s|%(name)s|%(message)s\033[1m')
        ch.setFormatter(formatter)
        self.log.addHandler(ch)

        self.dimensions = {
            'L': 0.075,
            'R': 0.035
        }

        self.power_left, self.power_right = 0, 0

        self._frames = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10),
            'theta': deque(maxlen=10)
        }

        self.vx, self.vy, self.vtheta = 0, 0, 0
        self.x, self.y, self.theta = 0, 0, 0
        self.speed = 0
        self.last_frame = 0
        self.actual_frame = 0

    def start(self):
        self.strategy.start(self)

    def get_name(self):
        return 'ROBOT_{}_{}'.format(self.robot_id, self.team_color)

    def is_visible(self):
        actual_frame = self.actual_frame
        robot_frame = self.last_frame
        if abs(actual_frame - robot_frame) > 45:
            return False
        return True

    def update(self, frame):
        team_color_key = 'robotsBlue' if self.team_color == 'blue' else 'robotsYellow'

        robot_data = [i for i in frame.get(team_color_key, []) if i.get('robotId') == self.robot_id]

        if len(robot_data) >= 1:
            self.current_data = robot_data[0]
        else:
            return

        self._update_speeds()
        self.update_stuckness()

    def get_speed(self):
        return (self.vx ** 2 + self.vy ** 2) ** .5

    def _update_speeds(self):
        self._frames['x'].append(self.current_data['x'])
        self._frames['y'].append(self.current_data['y'])
        self._frames['theta'].append(self.current_data['orientation'])

        self.theta = self.current_data['orientation']

        self.x = self.current_data['x']
        self.y = self.current_data['y']

        self.vx = avg_speed(self._frames['x'], self.game.vision._fps)
        self.vy = avg_speed(self._frames['y'], self.game.vision._fps)
        self.vtheta = angular_speed(self._frames['theta'], self.game.vision._fps)

        self.speed = math.sqrt(self.vx ** 2 + self.vy ** 2)

    def update_stuckness(self):
        MIN_STUCK_SPEED = 0.5
        MIN_STUCK_ANG_SPEED = 1

        if self.game.use_referee and not self.game.referee.can_play:
            self.stuck_time = 0

        if self.speed <= MIN_STUCK_SPEED and abs(self.vtheta) <= MIN_STUCK_ANG_SPEED:
            self.stuck_time += 1
        else:
            self.stuck_time = 0

    def is_stuck(self):
        MIN_STUCK_TIME = 1  # in seconds
        if self.game.vision._fps > 0:
            time_in_seconds = self.stuck_time / self.game.vision._fps
            if time_in_seconds > MIN_STUCK_TIME:
                return True
        return False

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
        if robot_world_speed[0] > 0.0:
            robot_world_speed[1] = -robot_world_speed[1]
            robot_world_speed[0] = -robot_world_speed[0]

        robot_angle_speed = -math.atan2(robot_world_speed[1], robot_world_speed[0])

        # va = signal of robot_angle_speed {-1, 1} * robot_world_speed.y [0, 1] * math.pi (max_speed = PI rad/s )
        va = (robot_angle_speed / abs(robot_angle_speed)) * robot_world_speed[1] * math.pi
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
        self.strategy.set_desired(desired)
        self.power_left, self.power_right = self.strategy.update()

        return self._get_command(self.power_left, self.power_right)

    def _get_command(self, power_left, power_right):
        return {
            'robot_id': self.robot_id,
            'wheel_left': power_left,
            'wheel_right': power_right,
            'color': self.team_color
        }
    

    def update_information(self, **kwargs): #Function to update values recieved in api
        for key, value in kwargs.items():
            if hasattr(self, key.lower()):
                setattr(self, key.lower(), value)

    def __getitem__(self, item):
        if item == 0:
            return self.x

        if item == 1:
            return self.y

        if item == 2:
            return self.theta

        raise IndexError("Robot only has 3 coordinates")
