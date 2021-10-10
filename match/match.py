import os
import time
import entities

from concurrent import futures

CATEGORIES = {
    '3v3': 3, '5v5': 5
}

class Match(object):
    def __init__(self, game, team_color, coach_name=None, category="3v3"):
        super().__init__()
        self.game = game
        
        self.coach_name = os.environ.get('COACH_NAME', coach_name) 
        self.team_color = os.environ.get('TEAM_COLOR', team_color)
        self.category = os.environ.get('CATEGORY', category)
        self.n_robots = CATEGORIES.get(self.category)

        self.opposite_team_color = 'yellow' if self.team_color == 'blue' else 'blue'

    
    def start(self):
        print("Starting match module starting ...")
        self.ball = entities.Ball(self.game)

        self.opposites = [
            entities.Robot(self.game, i, self.opposite_team_color) for i in range(self.n_robots)
        ]

        self.robots = [
            entities.Robot(self.game, i, self.team_color) for i in range(self.n_robots)
        ]

        self.coach = entities.coach.COACHES[self.coach_name](self)
        print(f"Match started! coach is [{self.coach.NAME}]")
        self.coach.decide()

        for robot in self.robots:
            robot.start()

    def update(self, frame):
        self.ball.update(frame)

        for entity in self.opposites:
            entity.update(frame)
        
        for entity in self.robots:
            entity.update(frame)


    def decide(self):
        commands = []
        commands_futures = []
        '''
        https://docs.python.org/3/library/concurrent.futures.html
        '''
        self.coach.decide()

        with futures.ThreadPoolExecutor(max_workers=self.n_robots) as executor:
            commands_futures = [
                executor.submit(robot.decide) for robot in self.robots
            ]

        for future in futures.as_completed(commands_futures):
            commands.append(future.result())

        return commands
