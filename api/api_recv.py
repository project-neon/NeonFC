from socket import *
import json
import threading
import struct

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


            #List with informations that are needed to be recieved and changed with there are any modifications.
            needs = [['TEAM_COLOR',self.match.team_color], ['TEAM_SIDE', self.match.team_side], ['GAME_STATUS',self.match.game_status], 
                     ['COACH_NAME', self.match.coach_names]] 


            for i in needs:
                info = decoded_data.get(i[0], None)

                if info and info != i[1]:
                    i[1] = info #Changing informations on other files if any are needed

                

            # team_color = decoded_data.get('TEAM_COLOR', None)
            # team_side = decoded_data.get('TEAM_SIDE', None)
            # game_status = decoded_data.get('GAME_STATUS', None)

            # # Change team color - Doesnt need to return color because it already changes on match team color inside the function
            # if team_color and team_color != self.match.team_color:
            #     self.match.restart(team_color)  
            
            # # Stop game and game on
            # if game_status and game_status != self.match.game_status:
            #     self.match.game_status = game_status

            # # Change team side - returns team side
            # if team_side and team_side != self.match.team_side:
            #     self.match.team_side = team_side



            self.decod_data = decoded_data

#Oq o NeonFC vai receber: 
# 
# match: cor do time, status do jogo, lado do time, coach
# 
# referee
# 
# robots: parametros