import comm
import vision
import match
import argparse
import fields as pitch
from commons.utils import get_config

parser = argparse.ArgumentParser(description='NeonFC')

parser.add_argument('--config_file', default='config_lab.json')

args = parser.parse_args()

class Game():
    def __init__(self, config_file=None):
        self.config = get_config(config_file)

        self.match = match.LabMatch(self,
            **self.config.get('match')
        )
        self.vision = vision.FiraVision()
        self.comm = comm.FiraFullComm()
        self.field = pitch.Field(self.match.category)

        self.use_referee = False
        
        self.start()

    def start(self):
        self.vision.assign_vision(self)
        
        self.vision.start()
        self.comm.start()
        self.match.start()
        

    def update(self):
        frame = vision.assign_empty_values(
            self.vision.frame, 
            field_size=self.field.get_dimensions(),
            team_side=self.match.team_side
        )

        self.match.update(frame)
        commands = self.match.decide()
        self.comm.send(commands)


g = Game(config_file=args.config_file)
