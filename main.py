import os
import api
import comm
import vision
import match
import argparse
from fields.fields import Field
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
        self.referee = comm.RefereeComm()
        self.data_sender = api.DataSender()
        self.field = Field(self.match.category)

        if os.environ.get('USE_DATA_SENDER'):
            self.use_data_sender = bool(int(os.environ.get('USE_DATA_SENDER')))
        else:
            self.use_data_sender = self.config.get('data_sender', False)
        
        if os.environ.get('USE_REFEREE'):
            self.use_referee = bool(int(os.environ.get('USE_REFEREE')))
        else:
            self.use_referee = self.config.get('referee')
        
        self.start()

    def start(self):
        self.vision.assign_vision(self)
        self.match.start()

        self.vision.start()
        self.comm.start()
        self.referee.start()
        self.data_sender.start()

    def update(self):
        frame = vision.assign_empty_values(
            self.vision.frame, 
            color=self.match.team_color,
            field_size=self.field.get_field()
        )
        self.match.update(frame)
        commands = self.match.decide()

        if self.referee.can_play or (not self.use_referee):
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
        
        if self.use_data_sender:
            api.DataSender().send_data()

g = Game(config_file=args.config_file)
