import os
from api.new_api import Api
from api.api_recv import Api_recv
import comm
import vision
import match
import argparse
import fields as pitch
from pyVSSSReferee.RefereeComm import RefereeComm
from commons.utils import get_config

parser = argparse.ArgumentParser(description='NeonFC')
parser.add_argument('--config_file', default='config.json')

args = parser.parse_args()

class Game():
    def __init__(self, config_file=None):
        self.config = get_config(config_file)
        self.match = match.Match(self,
            **self.config.get('match')
        )
        self.vision = vision.FiraVision()
        self.comm = comm.FiraComm()
        self.field = pitch.Field(self.match.category)
        self.referee = RefereeComm(config_file)

        self.use_api = self.config.get("api")
        self.api_address = self.config.get("network").get("api_address")
        self.api_port = self.config.get("network").get("api_port")
        self.api_recv_port = self.config.get("network").get("api_recv_port")

        self.api = Api(self.api_address, self.api_port)

        self.api_recv = Api_recv(self.match, self.api_address, self.api_recv_port)
        
        if os.environ.get('USE_REFEREE'):
            self.use_referee = bool(int(os.environ.get('USE_REFEREE')))
        else:
            self.use_referee = self.config.get('referee')
        
        self.start()

    def start(self):
        self.vision.assign_vision(self)
        self.referee.start()
        self.match.start()
        
        self.vision.start()
        self.comm.start()

        if self.use_api:
            self.api.start()
            self.api_recv.start()
        

    def update(self):
        frame = vision.assign_empty_values(
            self.vision.frame, 
            field_size=self.field.get_dimensions(),
            team_side=self.match.team_side
        )
        self.match.update(frame)
        commands = self.match.decide()

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

        if self.referee.can_play() or (not self.use_referee):
            self.comm.send(commands)
        else:
            commands = [
                {
                    'robot_id': r['robot_id'],
                    'color': r['color'],
                    'wheel_left': 0,
                    'wheel_right': 0
                } for r in commands
            ]
            self.comm.send(commands)

            if self.referee.get_foul() != "STOP" and self.referee.get_foul() != 7 and self.referee.get_foul() != None:
                if self.match.coach.get_positions( self.referee.get_foul(), self.match.team_color.upper(), self.referee.get_color(), self.referee.get_quadrant() ):
                    self.referee.send_replacement(
                        self.match.coach.get_positions( self.referee.get_foul(), self.match.team_color.upper(), self.referee.get_color(), self.referee.get_quadrant() ),
                        self.match.team_color.upper()
                    )

        if self.use_api:
            self.api.send_data(self.match)

g = Game(config_file=args.config_file)
