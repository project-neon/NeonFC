import os
import entities

from concurrent import futures

class Match(object):
    def __init__(self, game, team_color, num_robots=3, coach_name=None, category="3v3"):
        super().__init__()
        self.game = game
        self.n_robots = num_robots
        self.coach_name = coach_name
        self.team_color = os.environ.get('TEAM_COLOR', team_color)
        self.category = category

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
        '''
        https://docs.python.org/3/library/concurrent.futures.html
        '''

        self.coach.decide()

        with futures.ThreadPoolExecutor(max_workers=self.n_robots) as executor:
            commands = [
                executor.submit(robot.decide).result() for robot in self.robots
            ]
        
        return commands
