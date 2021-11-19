import os
import json
import struct
import socket
import threading

from commons.utils import get_config

from google.protobuf.json_format import MessageToJson
from protocols import command_pb2, packet_pb2, vssref_command_pb2

class FiraComm(object):
    def __init__(self, env, ):
        super(FiraComm, self).__init__()
        self.config = get_config()

        self.env = env

        self.commands = []
    
    def start(self):
        print("no need for communication!")
    
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
        action = [
            robot_commands[0]["wheel_left"],
            robot_commands[0]["wheel_right"],
            robot_commands[1]["wheel_left"],
            robot_commands[1]["wheel_right"],
            robot_commands[2]["wheel_left"],
            robot_commands[2]["wheel_right"]
        ]

        next_state, reward, done, _ = self.env.step(action)

    def _get_robot_color(self, robot):
        return True if robot['color'] == 'yellow' else False
    
    def _create_socket(self):
        return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class RefereeComm(threading.Thread):
        def __init__(self):
            super(RefereeComm, self).__init__()
            self.config = get_config()
            self.commands = []

            self.status = None

            self.referee_port = int(os.environ.get('REFEREE_PORT', self.config['network']['referee_port']))
            self.host = os.environ.get('REFEREE_IP', self.config['network']['referee_ip'])

            self.can_play = False
        
        def run(self):
            print("Starting referee...")
            self.referee_sock = self._create_socket()
            print("Referee completed!")
            while True:
                c = vssref_command_pb2.VSSRef_Command()
                data = self.referee_sock.recv(1024)
                c.ParseFromString(data)
                self.status = json.loads(MessageToJson(c))

                self.can_play = self.status.get('foul') == 'GAME_ON'

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

            sock.bind((self.host, self.referee_port))

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


if __name__ == "__main__":
    import time
    c = FiraComm()

    c.start()

    while True:
        time.sleep(1)
        c.send(
            [
                {
                    'robot_id': 0,
                    'wheel_left': 20,
                    'wheel_right': -20,
                    'color': 'blue'
                }
            ]
        )