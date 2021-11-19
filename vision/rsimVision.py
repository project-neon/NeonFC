import socket
import struct

import json
import time
import math

import threading

import numpy as np

from collections import deque
from commons.utils import get_config


class RSimVision(threading.Thread):
    def __init__(self, env):
        super(RSimVision, self).__init__()
        self.config = get_config()

        self.frame = {}

        self.env = env
        self.next_state = None

        self._fps = 0
        self._frame_times = deque(maxlen=120)

    def assign_vision(self, game):
        self.game = game

    def set_fps(self):
        self._frame_times.append(time.time())
        if len(self._frame_times) <= 3:
            return
        fps_frame_by_frame = [
            (v - i) for i, v in zip(self._frame_times, list(self._frame_times)[1:])
        ]
        self._fps = len(fps_frame_by_frame)/sum(fps_frame_by_frame)

    def run(self):
        if self.frame == {}:
            frame = self.env.reset()
            print("Rsoccer Vision completed!")
            self.frame = self.env.rsim.get_frame()

        while True:
            self.game.update()
            if self.done:
                frame = self.env.reset()
            if self.next_state is not None:
                self.frame = self.env.rsim.get_frame()
                self.set_fps()
    
    def send(self, robot_commands = []):
        '''
        Send commands to FIRASim

        robot_commands follows:
        [
            {
                robot_id: NUM,
                color: 'yellow|blue',
                wheel_left: float,
                wheel_right: float,
            }
        ]
        '''
        robot_commands = sorted(robot_commands, key=lambda k: k['robot_id'])
        action = [
            [
                robot_commands[i]["wheel_left"]/100,
                robot_commands[i]["wheel_right"]/100
            ] for i in range(0, 3)
        ]

        self.next_state, self.reward, self.done, _ = self.env.step(action)
        self.env.render()


def cleaning_data(raw_frame, color, field_size):
    w, h = field_size
    frame = {}

    _fm = -1 if color == 'yellow' else 1 # field multiplier
    _ap = math.pi if color == 'yellow' else 0 # angle part

    frame['ball'] = {}
    frame['ball']['x'] = _fm * raw_frame.ball.x + w/2
    frame['ball']['y'] = _fm * raw_frame.ball.y + h/2

    frame['robotsBlue'] = []
    frame['robotsYellow'] = []

    for _id, robot in raw_frame.robots_yellow.items():
        robot_bundle = {}
        robot_bundle['x'] = _fm * robot.x + w/2
        robot_bundle['y'] = _fm * robot.y + h/2
        robot_bundle['orientation'] = math.radians(robot.theta) + _ap
        robot_bundle['robotId'] = _id

        frame['robotsYellow'].append(robot_bundle)
    
    for _id, robot in raw_frame.robots_blue.items():
        robot_bundle = {}
        robot_bundle['x'] = _fm * robot.x + w/2
        robot_bundle['y'] = _fm * robot.y + h/2
        robot_bundle['orientation'] = math.radians(robot.theta) + _ap
        robot_bundle['robotId'] = _id

        frame['robotsBlue'].append(robot_bundle)
    
    return frame
