import entities
import algorithims

from concurrent import futures

class Match(object):
    def __init__(self, game, team_color, num_robots=3):
        super().__init__()
        self.game = game
        self.n_robots = num_robots
        self.team_color = team_color

        self.opposite_team_color = 'yellow' if team_color == 'blue' else 'blue'
    
    def start(self):
        self.ball = entities.Ball(self.game)

        self.robots = [
            entities.Robot(self.game, i, self.team_color) for i in range(self.n_robots)
        ]

        self.opposites = [
            entities.Robot(self.game, i, self.opposite_team_color) for i in range(self.n_robots)
        ]

        self.field = algorithims.DiscreteField()

    def update(self, frame):
        self.ball.update(frame)

        for entity in self.opposites:
            entity.update(frame)
        
        for entity in self.robots:
            entity.update(frame)
        
        self.field.update(
            avoiances=[
                {"x": r.x, "y": r.y, "radius": int(r.dimensions['L']/2)} for r in self.robots
            ]
        )


        
    def decide(self):
        commands = []
        '''
        https://docs.python.org/3/library/concurrent.futures.html
        '''
        with futures.ThreadPoolExecutor(max_workers=self.n_robots) as executor:
            commands = [
                executor.submit(robot.decide).result() for robot in self.robots
            ]
        
        return commands