from socket import *
import json
import time
import threading


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
    
    #sender_thread: threading.Thread
    
    def __init__(self, address, port):
        self.address = address
        self.port = port

    # Initiate socket connection
    def start(self):
        self.obj_socket = socket(AF_INET, SOCK_DGRAM)

    # Sends dict game data to socket listener
    def send_data(self, info_api):
         while True:
            data_dict = info_api.organize_send()
            msg = json.dumps(data_dict)
            self.obj_socket.sendto(msg.encode(), (self.address, self.port)) #Problema 2

    
    def send_custom_data(self, data):
         msg = json.dumps(data)
         self.obj_socket.sendto(msg.encode(), (self.address, self.port))