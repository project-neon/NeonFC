import socket
import struct
import json
import time
import math
import threading
from collections import deque
from commons.utils import get_config
from google.protobuf.json_format import MessageToJson
from protocols.ssl_vision import messages_robocup_ssl_wrapper_pb2
import os

class SSLVision(threading.Thread):
    def __init__(self):
        super(SSLVision, self).__init__()
        self.config = get_config('config_real_life.json')

        self.frame = {}
        self.last_frame = {}
        
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
        # print(self._fps)

    def run(self):
        print("Starting vision...")
        self.vision_sock = self._create_socket()
        self._wait_to_connect()
        print("Vision completed!")

        while True:
            env = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
            data = self.vision_sock.recv(1024)
            self.set_fps()
            env.ParseFromString(data)
            self.frame = json.loads(MessageToJson(env))

            self.game.update()
            
    
    def _wait_to_connect(self):
        self.vision_sock.recv(1024)
    
    def _create_socket(self):
        print(f"Creating socket with address: {self.host} and port: {self.vision_port}")
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


def assign_empty_values(raw_frame, field_size, team_side, last_frame=None):
    if raw_frame.get('detection') is None:
        return last_frame

    frame = raw_frame.get('detection')
    w, h = field_size

    frame['ball'] = {}
    if frame.get('balls'):
        
        if team_side == 'right':
            frame['ball']['x'] = -frame['balls'][0].get('x', 0)
            frame['ball']['y'] = -frame['balls'][0].get('y', 0)
        else:
            frame['ball']['x'] = frame['balls'][0].get('x', 0)
            frame['ball']['y'] = frame['balls'][0].get('y', 0)

        frame['ball']['x'] = frame['ball']['x']  / 1000 + w/2
        frame['ball']['y'] = frame['ball']['y'] / 1000 + h/2
    else:
        # TODO: definir como vamos tratar quando nao ha um elemento necessario no campo como a bola
        frame['ball']['x'] = -1
        frame['ball']['y'] = -1
    
    for robot in frame.get("robotsYellow", []):
        if team_side == 'right':
            robot['x'] = - robot.get('x', 0)
            robot['y'] = - robot.get('y', 0)
            robot['orientation'] = robot.get('orientation', 0) + math.pi

        robot['x'] = robot.get('x', 0)  / 1000 + w/2
        robot['y'] = robot.get('y', 0)  / 1000 + h/2
        robot['robotId'] = robot.get('robotId', 0)
        robot['orientation'] = robot.get('orientation', 0)
    
    for robot in frame.get("robotsBlue", []):
        if team_side == 'right':
            robot['x'] = - robot.get('x', 0)  
            robot['y'] = - robot.get('y', 0) 
            robot['orientation'] = robot.get('orientation', 0) + math.pi

        robot['x'] = robot.get('x', 0)  / 1000 + w/2
        robot['y'] = robot.get('y', 0)  / 1000+ h/2
        robot['robotId'] = robot.get('robotId', 0)
        robot['orientation'] = robot.get('orientation', 0)

    return frame

if __name__ == "__main__":
    import time
    v = SSLVision()

    v.start()

    while True:
        time.sleep(1)
        print(v.frame)
