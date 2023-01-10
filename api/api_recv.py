from socket import *
import json
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
            decoded_data = json.loads(data.decode())
            # Feedback commands from socket (e.g. an interface)
            #print(decoded_data)

            team_color = decoded_data.get('TEAM_COLOR')
            team_side = decoded_data.get('TEAM_SIDE')
            game_status = decoded_data.get('GAME_STATUS')

            # Change team color - Doesnt need to return color because it already changes on match team color inside the function
            if team_color != self.match.team_color:
                self.match.restart(team_color)
            
            # Stop game and game on
            if game_status != self.match.game_status:
                self.match.game_status = game_status

            # Change team side - returns team side
            if team_side != self.match.team_side:
                self.match.team_side = team_side
