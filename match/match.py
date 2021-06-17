import os
import time
import entities
import algorithms

from concurrent import futures

AVAILABLE_COACHES = {
    'LARC2020Coach': entities.coach.LarcCoach,
    'Iron2021Coach': entities.coach.IronCoach,
    'Astar': entities.coach.AstarCoach,
    'devAlex': entities.coach.AlexCoach
}

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
        self.ball = entities.Ball(self.game)

        self.opposites = [
            entities.Robot(self.game, i, self.opposite_team_color) for i in range(self.n_robots)
        ]

        self.robots = [
            entities.Robot(self.game, i, self.team_color) for i in range(self.n_robots)
        ]

        self.coach = AVAILABLE_COACHES[self.coach_name](self)
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
        t_init = time.time()
        self.coach.decide()

        with futures.ThreadPoolExecutor(max_workers=self.n_robots) as executor:
            commands_futures = [
                executor.submit(robot.decide) for robot in self.robots
            ]

        for future in futures.as_completed(commands_futures):
            commands.append(future.result())
      
        print(time.time() - t_init)
        return commands
