from algorithims import PID

class Robot(object):
    def __init__(self, robot_id, team_color):
        super().__init__()

        self.strategy = {} # 'NEEDS IMPLEMENTATION'
        self.robot_id = robot_id
        self.team_color = team_color
        self.data = {}

        self.pid = PID.RobotPid()

    def set_strategy(self, strategy):
        self.strategy = strategy

    def set_data_from_frame(self, frame):
        team_color_key = 'robotsBlue' if self.team_color == 'blue' else 'robotsYellow'

        robot_data = [i for i in frame[team_color_key] if i.get('robot_id') == self.robot_id]

        if len(robot_data) >= 1:
            self.data = robot_data[0]
        else:
            print('## CAUTION: robo não encontrado')
    
    def decide(self):
        speed_y, speed_theta = 20, 5

        self.pid.set_target(speed_y, speed_theta)
        
        power_left, power_right = self.pid.speed_to_power()
