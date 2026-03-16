
class Controller(object):
    v = 0.0; w = 0.0

    def __init__(self, robot):
        self.robot = robot
        self.environment = robot.game.environment
        self.match = robot.game.match
        self.game = robot.game

    def set_desired(self, desired):
        self.v = desired[0]
        self.w = desired[1]

    def update(self):
        return self.v, self.w
