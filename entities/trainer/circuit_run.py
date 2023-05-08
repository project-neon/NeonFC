from entities.trainer.base_trainer import Trainer
import strategy


class CircuitRunTrainer(Trainer):
    NAME = 'TEST'
    def __init__(self, lab_match, n_iterations = 1, train_name="CircuitRunning"):
        super().__init__(lab_match, n_iterations, train_name)

        self.robot = self.match.essay_robot

        fx, fy = self.match.game.field.get_dimensions()[0]/2, self.match.game.field.get_dimensions()[1]/2

        self.circuit = [
            (0.370 + fx, 0.430 + fy),
            (0.370 + fx, 0 + fy),
            (0.370 + fx, -0.430 + fy),

            (0 + fx, -0.430 + fy),

            (-0.370 + fx, -0.430 + fy),
            (-0.370 + fx, 0 + fy),
            (-0.370 + fx, 0.430 + fy),

            (0 + fx, 0.430 + fy),
        ]

        self.train_strategy = strategy.tests.UVF_Test(self.match)

        # self.train_strategy.set_circuit(self.circuit)

        self.idle_strategy = strategy.tests.Idle(self.match)


    def execute(self):
        robots = self.match.robots

        for robot in robots:
            if robot.strategy is not None:
                continue

            if robot.robot_id == 0:
                robot.strategy = self.train_strategy
            else:
                robot.strategy = self.idle_strategy
            
            robot.start()

    def setup(self):
        super().setup()
        
        self.repositioning_robot(self.robot, [0, 0, 0])
        self.execute()
