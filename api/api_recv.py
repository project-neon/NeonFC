from socket import *
import json
import threading
import struct
import api.Info_api as Info_api

class Api_recv(threading.Thread):
    def __init__(self, match, address, port):
        super(Api_recv, self).__init__()

        BUFFER_SIZE = 2048

        self.match = match
        self.address = address
        self.port = port
        self.buffer_size = BUFFER_SIZE
        self.decod_data = None   

    # Receives data
    def run(self):
        self.obj_socket = socket(AF_INET, SOCK_DGRAM)
        self.obj_socket.bind((self.address, self.port))

        print("Starting api_recv...")

        while True:
            data, origem = self.obj_socket.recvfrom(self.buffer_size)
            decoded_data = json.loads(data.decode())
            # Feedback commands from socket (e.g. an interface)
            #print(decoded_data)

            Info_api.update_recv(decoded_data)

            self.decod_data = decoded_data