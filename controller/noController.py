from controller.Controller import Controller


class NoController(Controller):
    def __init__(self, robot):
        super().__init__(robot)

    def set_desired(self, desired):
        self.v = desired[0]
        self.w = desired[1]

    def update(self):
        return self.v, self.w
