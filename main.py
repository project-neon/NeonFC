import comm
import vision
import match

from commons.utils import get_config


class Game():
    def __init__(self):
        self.config = get_config()

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
        frame = vision.assign_empty_values(self.vision.frame)
        
        self.match.update(frame)

        subject_command = self.match.decide()

        self.comm.send(subject_command)



g = Game()