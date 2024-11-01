from socket import *
import json
import threading
import struct
import time, collections

BUFFER_SIZE = 4096

class Api_recv(threading.Thread):
    def __init__(self, match, address, port):
        super(Api_recv, self).__init__()

        self.match = match
        self.address = address
        self.port = port
        self.buffer_size = BUFFER_SIZE
        self.decod_data = None   

        self.kill_recieved = False
        self.t1 = time.time()
        self.list = collections.deque(maxlen=25)

    def connect_info(self,info_api):
        self.info_api = info_api

    # Receives data
    def run(self):
        # self.obj_socket = socket(AF_INET, SOCK_DGRAM)
        self.obj_socket = socket(AF_INET, SOCK_STREAM)
        self.obj_socket.setsockopt(SOL_SOCKET, SO_KEEPALIVE, 1)
        self.obj_socket.bind((self.address, self.port))
        self.obj_socket.listen(1)
        print("Starting api_recv...")

        while not self.kill_recieved:
            connection, addr = self.obj_socket.accept()
            while True:
                data = connection.recv(BUFFER_SIZE)
                
                if data:
                    # data, origem = self.obj_socket.recvfrom(self.buffer_size)
                    decoded_data = json.loads(data.decode())
                    # Feedback commands from socket (e.g. an interface)

                    self.info_api.update_recv(decoded_data)

                    self.decod_data = decoded_data
