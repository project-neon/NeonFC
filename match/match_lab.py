import os
import entities
from api import Api

from concurrent import futures

CATEGORIES = {
    '3v3': 3, '5v5': 5
}

class LabMatch(object):
    def __init__(self, game, team_side, team_color, train_name=None, category="3v3"):
        super().__init__()
        self.game = game
        
        self.train_name = os.environ.get('TRAIN_NAME', train_name) 
        self.team_side = os.environ.get('TEAM_SIDE', team_side) 
        self.team_color = os.environ.get('TEAM_COLOR', team_color)
        self.category = os.environ.get('CATEGORY', category)

        self.n_robots = CATEGORIES.get(self.category)

        self.opposite_team_color = 'yellow'

        self.game_status = 'stop'

    
    def start(self):
        print("Starting match module starting ...")
        self.ball = entities.Ball(self.game)

        self.opposites = [
            entities.Robot(self.game, i, self.opposite_team_color) for i in range(self.n_robots)
        ]

        self.robots = [
            entities.Robot(self.game, i, self.team_color) for i in range(self.n_robots)
        ]

        self.essay_robot = self.robots[0] # will awalys be zero from the main team

        self.train = entities.trainer.TRAINERS[self.train_name](self)

        print(f"Essay started! traning is is [{self.train.NAME}]")

        self.train.setup()

        self.essay_robot.start()

    def restart(self, team_color):
        pass


    def update(self, frame):
        self.ball.update(frame)

        for entity in self.opposites:
            entity.update(frame)
        
        for entity in self.robots:
            entity.update(frame)


    def decide(self):
        commands = []
        self.train.decide()
        commands.append(self.essay_robot.decide())
        return commands
