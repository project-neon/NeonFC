import comm
import vision
import match
import argparse

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
        
        self.start()

    def start(self):
        self.vision.assign_vision(self)
        self.match.start()
        self.vision.start()
        self.comm.start()

    def update(self):
        frame = vision.assign_empty_values(
            self.vision.frame, 
            color=self.config['match']['team_color']
        )
        self.match.update(frame)
        subject_command = self.match.decide()
        self.comm.send(subject_command)

g = Game(config_file=args.config_file)