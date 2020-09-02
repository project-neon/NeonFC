import comm
import vision
import entities

import concurrent.futures

class Game():
    def __init__(self):
        self.vision = vision.FiraVision()
        self.comm = comm.FiraComm()
        self.start()

    def start(self):
        self.vision.assign_vision(self)

        self.vision.start()
        self.comm.start()

        self.test_subject = entities.Robot(self, 0, 'blue')
        self.ball = entities.Ball(self)
    
    def run(self):
        '''
        https://docs.python.org/3/library/concurrent.futures.html
        '''
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            robot_commands = {executor.submit()}


    def update(self):
        frame = vision.assign_empty_values(self.vision.frame)
        # print([a for a in frame.get('robotsBlue') if a['robotId'] == 0])

        self.ball.update(frame)
        self.test_subject.update(frame)
        
        subject_command = self.test_subject.decide()

        self.comm.send(
            [subject_command]
        )



g = Game()