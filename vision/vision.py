import socket
import struct

import json
import time
import math

import threading

import os

from collections import deque
from commons.utils import get_config

from google.protobuf.json_format import MessageToJson

from protocols import packet_pb2


class FiraVision(threading.Thread):
    def __init__(self):
        super(FiraVision, self).__init__()
        self.config = get_config()

        self.frame = {}
        
        self.vision_port = int(os.environ.get('VISION_PORT', self.config['network']['vision_port']))
        self.host = os.environ.get('MULTICAST_IP', self.config['network']['multicast_ip'])

        self._fps = 0
        self._frame_times = deque(maxlen=60)

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
        print("Starting vision...")
        self.vision_sock = self._create_socket()
        self._wait_to_connect()
        print("Vision completed!")

        while True:
            env = packet_pb2.Environment()
            data = self.vision_sock.recv(1024)
            self.set_fps()
            env.ParseFromString(data)
            self.frame = json.loads(MessageToJson(env))
            self.game.update()
            
    
    def _wait_to_connect(self):
        self.vision_sock.recv(1024)
    
    def _create_socket(self):
        sock = socket.socket(
            socket.AF_INET, 
            socket.SOCK_DGRAM, 
            socket.IPPROTO_UDP
        )

        sock.setsockopt(
            socket.SOL_SOCKET, 
            socket.SO_REUSEADDR, 1
        )

        sock.bind((self.host, self.vision_port))

        mreq = struct.pack(
            "4sl",
            socket.inet_aton(self.host),
            socket.INADDR_ANY
        )

        sock.setsockopt(
            socket.IPPROTO_IP, 
            socket.IP_ADD_MEMBERSHIP, 
            mreq
        )

        return sock


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

if __name__ == "__main__":
    import time
    v = FiraVision()

    v.start()

    while True:
        time.sleep(1)
        print(v.frame)