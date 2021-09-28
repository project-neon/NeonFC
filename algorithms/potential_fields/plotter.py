import json

class PotentialDataExporter(object):
    def __init__(self, name):
        self.file = open(
            '{}|potential_field.log'.format(name),
            'w'
        )

    def export(self, behaviour, robot, ball):
        X = []
        Y = []
        U = []
        V = []

        for x in range(-10, 150 + 10, 2):
            x = x/100.0
            for y in range(-10, 130 + 10, 2):
                y = y/100.0
                robot.x = x 
                robot.y = y
                res = behaviour.compute([x, y])
                X.append(x)
                Y.append(y)
                U.append(res[0])
                V.append(res[1])

        plot_file = {
            "x": X,
            "y": Y,
            "u": U,
            "v": V,
            "robot_x": robot.x,
            "robot_y": robot.y,
            "ball_x": ball.x,
            "ball_y": ball.y,
            "behaviour": behaviour.name
        }

        self.file.write(json.dumps(plot_file) + "||")