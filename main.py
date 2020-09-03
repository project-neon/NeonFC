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

        self.vision.start()
        self.comm.start()

        self.match.start()

    def update(self):
        frame = vision.assign_empty_values(self.vision.frame)
        # print([a for a in frame.get('robotsBlue') if a['robotId'] == 0])
        
        self.match.update(frame)
        # self.ball.update(frame)
        # self.test_subject.update(frame)
        
        subject_command = self.match.decide()

        self.comm.send(subject_command)



g = Game()