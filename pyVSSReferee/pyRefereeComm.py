import abc
import os
import json
import struct
import socket
import threading

from commons.utils import get_config

from google.protobuf.json_format import MessageToJson
from protocols import command_pb2, vssref_command_pb2

class RefereeComm(threading.Thread):
    @abc.abstractmethod
    def __init__(self):
        super(RefereeComm, self).__init__()
        self.config = get_config()
        self.commands = []

        self.status = None

        self.referee_port = int(os.environ.get('REFEREE_PORT', self.config['network']['referee_port']))
        self.host = os.environ.get('REFEREE_IP', self.config['network']['referee_ip'])

        self.can_play = False

    @abc.abstractmethod
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

    @abc.abstractmethod
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

