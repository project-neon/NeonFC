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
                robot_commands[i]["wheel_left"],
                robot_commands[i]["wheel_right"]
            ] for i in range(0, 3)
        ]

        self.next_state, self.reward, self.done, _ = self.env.step(action)


def assign_empty_values(raw_frame, color, field_size):
    frame = raw_frame.get('frame')
    w, h = field_size
    if frame.get('ball'):
        if color == 'yellow':
            frame['ball']['x'] = -frame['ball'].get('x', 0)
            frame['ball']['y'] = -frame['ball'].get('y', 0)

        frame['ball']['x'] = frame['ball'].get('x', 0) + w/2
        frame['ball']['y'] = frame['ball'].get('y', 0) + h/2
    
    for robot in frame.get("robotsYellow"):
        if color == 'yellow':
            robot['x'] = - robot.get('x', 0)
            robot['y'] = - robot.get('y', 0)
            robot['orientation'] = robot.get('orientation', 0) + math.pi

        robot['x'] = robot.get('x', 0) + w/2
        robot['y'] = robot.get('y', 0) + h/2
        robot['robotId'] = robot.get('robotId', 0)
        robot['orientation'] = robot.get('orientation', 0)
    
    for robot in frame.get("robotsBlue"):
        if color == 'yellow':
            robot['x'] = - robot.get('x', 0)
            robot['y'] = - robot.get('y', 0)
            robot['orientation'] = robot.get('orientation', 0) + math.pi

        robot['x'] = robot.get('x', 0) + w/2
        robot['y'] = robot.get('y', 0) + h/2
        robot['robotId'] = robot.get('robotId', 0)
        robot['orientation'] = robot.get('orientation', 0)
    
    return frame
