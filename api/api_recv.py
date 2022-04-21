from socket import *

from numpy import mat

from concurrent import futures

import json
import struct

import threading

class Api_recv(threading.Thread):
    def __init__(self, match, address, port):
        super(Api_recv, self).__init__()

        BUFFER_SIZE = 2048

        self.match = match

        self.address = address
        self.port = port
        self.buffer_size = BUFFER_SIZE
        

    # Receives data
    def run(self):
        self.obj_socket = socket(AF_INET, SOCK_DGRAM)
        self.obj_socket.bind((self.address, self.port))

        print("Starting api_recv...")

        while True:
            data, origem = self.obj_socket.recvfrom(self.buffer_size)
            
            decoded_data = data.decode()
            # Feedback commands from socket (e.g. an interface)
            print("UDP ME", decoded_data)

            game_status = decoded_data

            # Change team color - Doesnt need to return color because it already changes on match team color inside the function
            
            # Stop game and game on
            if game_status != self.match.game_status:
                self.match.game_status = game_status
