class NoController(object):
    def __init__(self, robot):
        self.robot = robot
        self.environment = robot.game.environment
        self.match = robot.game.match

    def set_desired(self, desired):
        self.v = desired[0]
        self.w = desired[1]

    def update(self):
        print(self.v)
        return self.v, self.w