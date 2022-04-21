import os
from api.new_api import Api
from api.api_recv import Api_recv
import comm
import vision
import match
import argparse
import fields as pitch
from commons.utils import get_config
from pyVSSSReferee.RefereeComm import RefereeComm
from vision.sslvision import assign_empty_values

parser = argparse.ArgumentParser(description='NeonFC')
parser.add_argument('--config_file', default='config_real_life.json')

args = parser.parse_args()

class Game():
    def __init__(self, config_file=None):
        self.config = get_config(config_file)
        self.match = match.MatchRealLife(self,
            **self.config.get('match')
        )
        self.vision = vision.SSLVision()
        self.comm = comm.RLComm()
        self.field = pitch.Field(self.match.category)

        self.use_api = self.config.get("api")
        self.api_address = self.config.get("network").get("api_address")
        self.api_port = self.config.get("network").get("api_port")
        self.api_recv_port = self.config.get("network").get("api_recv_port")

        self.referee = RefereeComm(config_file)

        self.api = Api(self.api_address, self.api_port)
        self.api_recv = Api_recv(self.match, self.api_address, self.api_recv_port)

        self.use_referee = False
        
        self.start()

    def start(self):
        self.vision.assign_vision(self)
        self.match.start()
        
        self.vision.start()
        self.comm.start()

        if self.use_api:
            self.api.start()
            self.api_recv.start()

            
        

    def update(self):
        frame = assign_empty_values(
            self.vision.frame, 
            field_size=self.field.get_dimensions(),
            team_side=self.match.team_side,
            last_frame=self.vision.last_frame
        )
        self.vision.last_frame = frame
        
        self.match.update(frame)
        commands = self.match.decide()

        self.comm.send(commands)
        pass

        print(f">>>>>>>>> ${self.match.game_status}<<<<<<<<<<<<<\n")

        if self.use_api and self.match.game_status == 'stop':
            commands = [
                {
                    'robot_id': r['robot_id'],
                    'color': r['color'],
                    'wheel_left': 0,
                    'wheel_right': 0
                } for r in commands
            ]
            self.comm.send(commands)


g = Game(config_file=args.config_file)
