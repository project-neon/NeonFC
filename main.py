import comm
import vision

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
        
    
    def run(self):
        '''
        https://docs.python.org/3/library/concurrent.futures.html
        '''
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            robot_commands = {executor.submit()}


    def update(self):
        frame = vision.assign_empty_values(self.vision.frame)



g = Game()