import time
import logging

class CustomFormatter(logging.Formatter):

    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "[%(asctime)s | %(name)s | %(levelname)s] %(message)s"

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

class Trainer(object):
    def __init__(self, lab_match, n_iterations, train_name):
        self.n_iterations = n_iterations
        self.train_name = train_name
        self.match = lab_match

        self.results = []

        self.logger = self.setup_logger(train_name)

    def setup_logger(self, train_name):
        logger = logging.getLogger(train_name)
        logger.setLevel(logging.DEBUG)

        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        ch.setFormatter(CustomFormatter())

        logger.addHandler(ch)
        return logger


    def repositioning_robot(self, r, pose):
        """
        pose as [x, y, theta]
        """
        command = {'x': pose[0], 'y': pose[1], 'theta': pose[2], 'color': r.team_color, 'robot_id': r.robot_id}

        self.logger.debug("reposicionando robo: " + str(command))

        self.match.game.comm.replace([command], None)
        time.sleep(2)
        self.logger.debug("posicionamento completo!")

    def clean_field(self):
        self.logger.debug("colocando robos e bola fora do campo...")
        commands = []
        ball_commands = {'x': -10, 'y': -10}
        for i, r in enumerate(self.match.robots + self.match.opposites):
            commands.append(
                {'x': 2, 'y': 2 + i * 0.1, 'theta': 0, 'color': r.team_color, 'robot_id': r.robot_id}
            )

        self.match.game.comm.replace(commands, ball_commands)
        time.sleep(2)
        self.logger.debug("posicionamento completo!")

    def decide(self):
        self.execute()

    def execute(self, iteration):
        pass

    def teardown(self, iteration):
        pass

    def setup(self):
        self.clean_field()

    def compile(self):
        pass