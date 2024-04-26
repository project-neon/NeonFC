from api import Api, Api_recv, InfoApi
import comm
import match
import argparse
import fields as pitch
from commons.utils import get_config
from pyVSSSReferee.RefereeComm import RefereeComm
from pySSLVision.VisionComm import SSLVision, assign_empty_values
import os
import threading
import time, collections

parser = argparse.ArgumentParser(description='NeonFC')
parser.add_argument('--config_file', default='config_real_life.json')
parser.add_argument('--env', default='real_life')

args = parser.parse_args()

class Game():
    def __init__(self, config_file=None, env='real_life'):
        self.config = get_config(config_file)
        self.match = match.MatchRealLife(self,
            **self.config.get('match')
        )
        self.vision = SSLVision()
        self.comm = comm.RLComm()
        self.field = pitch.Field(self.match.category)
        self.environment = env

        self.t1 = time.time()
        self.t2 = time.time() 
        self.list = collections.deque(maxlen=25)

        self.use_api = self.config.get("api")
        self.api_address = self.config.get("network").get("api_address")
        self.api_port = self.config.get("network").get("api_port")
        self.api_recv_port = self.config.get("network").get("api_recv_port")
 
        self.referee = RefereeComm(config_file)

        self.api = Api(self.api_address, self.api_port)
        self.api_recv = Api_recv(self.match, self.api_address, self.api_recv_port)

        if os.environ.get('USE_REFEREE'):
            self.use_referee = bool(int(os.environ.get('USE_REFEREE')))
        else:
            self.use_referee = self.config.get('referee')
        
        self.start()

    def start(self):
        self.vision.assign_vision(self.update)
        if self.use_referee:
            self.referee.start()
        self.match.start()

        self.vision.start()
        self.comm.start()

        if self.use_api:  
            #self.match.game_status = 'GAME_ON'   
            self.info_api = InfoApi(self.match, self.match.robots, self.match.opposites, self.match.coach, self.match.ball, self.match.parameters)
            self.api.start()
            self.api_recv.connect_info(self.info_api)
            self.api_recv.start()  #Problema 1

    def update(self):
        frame = assign_empty_values(
            self.vision.frame, 
            field_size=self.field.get_dimensions(),
            team_side=self.match.team_side,
            last_frame=self.vision.last_frame
        )
        self.vision.last_frame = frame
        
        self.match.update(frame)
        if self.use_referee:
            self.match.check_foul(self.referee)
        commands = self.match.decide()

        if (self.use_api or self.use_referee) and (self.match.game_status == 'STOP' or self.match.game_status is None):
            commands = [
                {
                    'robot_id': r['robot_id'],
                    'color': r['color'],
                    'wheel_left': 0,
                    'wheel_right': 0
                } for r in commands
            ]

        self.comm.send(commands)
        delta_t = float(time.time() - self.t1)
        self.list.append(delta_t)
        self.t1 = time.time()

        #print(len(self.list)/sum(self.list), 'hz')

        if self.use_api:
                self.api.send_data(self.info_api)
            
            
g = Game(config_file=args.config_file, env=args.env)
