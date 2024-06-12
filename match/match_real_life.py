import os
import entities
from concurrent.futures import ThreadPoolExecutor, as_completed, thread
from api import Parameter

CATEGORIES = {
    '3v3': 3, '5v5': 5
}

class MatchRealLife(object):
    def __init__(self, game, team_side, team_color, coach_name=None, category="3v3", robot_ids=[0,1,2], opposite_ids=[0,1,2]):
        super().__init__()
        self.game = game
        
        self.coach_name = os.environ.get('COACH_NAME', coach_name) 
        self.team_side = os.environ.get('TEAM_SIDE', team_side) 
        self.team_color = os.environ.get('TEAM_COLOR', team_color)
        self.category = os.environ.get('CATEGORY', category)
        self.n_robots = CATEGORIES.get(self.category)
        self.robot_ids = os.environ.get('robot_ids', robot_ids)
        self.opposite_ids = os.environ.get('opposite_ids', opposite_ids)
        

        pid_kp, ki, kd = Parameter(0.1, 'pid_kp'), Parameter(0, 'ki'), Parameter(0, 'kd')
        kw, rm, vm, uni_kp = Parameter(3.5, 'kw'), Parameter(0.44, 'rm'), Parameter(0.5, 'vm'), Parameter(1, 'uni_kp')
        
        self.parameters = {"pid_kp":pid_kp, "ki":ki, "kd":kd, "kw":kw, "rm":rm, "vm":vm, "uni_kp":uni_kp}

        self.opposite_team_color = 'yellow' if self.team_color == 'blue' else 'blue'

        self.game_status = 'STOP'
        self.match_event = {'event': 'PLAYING', 'quadrant': 1, 'mine': True}

    
    def start(self):
        print("Starting match module starting ...")
        self.ball = entities.Ball(self.game)

        self.opposites = [
            entities.Robot(self.game, i, self.opposite_team_color) for i in self.opposite_ids
        ]

        self.robots = [
            entities.Robot(self.game, i, self.team_color) for i in self.robot_ids
        ]

        self.coach = entities.coach.COACHES[self.coach_name](self)
        print(f"Match started! coach is [{self.coach.NAME}]")
        self.coach.decide()

        for robot in self.robots:
            robot.start()

    def restart(self, team_color):
        self.team_color = team_color
        self.opposite_team_color = 'yellow' if self.team_color == 'blue' else 'blue'

        self.opposites = [
            entities.Robot(self.game, i, self.opposite_team_color) for i in [] # range(self.n_robots)
        ]

        self.robots = [
            entities.Robot(self.game, i, self.team_color) for i in self.robot_ids
        ]

        self.coach.decide()

        for robot in self.robots:
            robot.start()


    def update(self, frame):
        self.ball.update(frame)

        for entity in self.opposites:
            entity.update(frame)
        
        for entity in self.robots:
            entity.update(frame)
        
    def check_foul(self, ref):
        if ref.can_play():
            self.match_event['event'] = 'PLAYING'
            self.game_status = 'GAME_ON'
        else:
            if ref.get_foul() == 'STOP':
                self.game_status = 'STOP'
                return
            self.game_status = 'GAME_ON'

            self.match_event['event'] = 'KICKOFF' if ref.get_foul() == None else ref.get_foul()
            self.match_event['quadrant'] = ref.get_quadrant()
            self.match_event['mine'] = ref.get_color() == self.team_color.upper()


    def update_information(self, info): #Function to update values recieved in api
        for key, value in info.items():
            setattr(self, key.lower(), value)
                

    def decide(self):
        commands = []
        commands_futures = []
        '''
        https://docs.python.org/3/library/concurrent.futures.html
        '''
        self.coach.decide()

        with ThreadPoolExecutor(max_workers=self.n_robots) as executor:
            commands_futures = [
                executor.submit(robot.decide) for robot in self.robots
            ]

        for future in as_completed(commands_futures):
            commands.append(future.result())

        return commands            