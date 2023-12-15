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


            #List with informations that are needed to be recieved and changed when there are any modifications.

            needs = [['TEAM_COLOR',self.match.team_color], ['TEAM_SIDE', self.match.team_side], ['GAME_STATUS',self.match.game_status]]
                     #['COACH_NAME', self.match.coach_names]] 
            
            needs_dict ={}

            for i in needs:
                info = decoded_data.get(i[0], None)

                if i[0] == 'TEAM_COLOR' and info != self.match.team_color:
                        self.match.restart(info) #This function already changes the color of our team, and accordingly, changes the color of the other team.
                elif i[0] != 'TEAM_COLOR':
                    needs_dict.update({i[0]: info})

            self.match.update_information(**needs_dict) #calls function in match.py to update values.


            #print('lista:', self.match.team_color, self.match.team_side, self.match.game_status)
            #print('variaveis:', self.match.team_color, self.match.team_side, self.match.game_status)



            self.match.update_information

            self.decod_data = decoded_data

#Oq o NeonFC vai receber: 
# 
# match: cor do time, status do jogo, lado do time, coach
# 
# referee
# 
# robots (lista no match, ver como chamar cada parametro): parametros