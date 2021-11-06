from socket import *

import json
#import socket
import struct

class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.
    source: https://refactoring.guru/design-patterns/singleton/python/example
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class Api(metaclass=SingletonMeta):
    def __init__(self):
        self.servidor = "127.0.0.1"
        self.porta = 43210

        self.obj_socket = socket(AF_INET, SOCK_DGRAM)
        self.obj_socket.connect((self.servidor, self.porta))
        self.saida = ""

    def recvData(self, obj):
        while self.saida != "X":
            msg = json.dumps(obj)
            self.obj_socket.sendto(msg.encode(), (self.servidor, self.porta))
            print(msg)
            dados, origem = self.obj_socket.recvfrom(65535) 
            print("Resposta do Servidor: ", dados.decode())
            self.saida = input("Digite <X> para sair: ").upper()

        self.obj_socket.close()
