class Match(object):
    def __init__(self, team_color, num_robots=3):
        super().__init__()

        self.n_robots = num_robots
        self.team_color = team_color
    
    def start(self):
        pass
