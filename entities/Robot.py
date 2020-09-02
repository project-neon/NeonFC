from collections import deque

def speed(_list, _fps):
    if len(_list) <= 1:
        return 0
    
    speed_fbf = [
        (v - i) for i, v 
        in zip(
            _list, 
            list(_list)[1:]
        )
    ]

    return _fps * (sum(speed_fbf)/len(speed_fbf)) 


class Robot(object):

    def __init__(self, game, robot_id, team_color):
        self.game = game
        self.robot_id = robot_id
        self.team_color = team_color
        self.current_data = {}

        self._frames = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10),
            'theta': deque(maxlen=10)
        }

        self.vx, self.vy, self.vtheta = 0, 0, 0
    
    def get_name(self):
        return 'ROBOT_{}_{}'.format(self.robot_id, self.team_color)

    def update(self, frame):
        team_color_key = 'robotsBlue' if self.team_color == 'blue' else 'robotsYellow'

        robot_data = [i for i in frame[team_color_key] if i.get('robotId') == self.robot_id]

        if len(robot_data) >= 1:
            self.current_data = robot_data[0]
        else:
            print('## CAUTION: robo não encontrado')
            raise ValueError('## CAUTION: robo não encontrado')
            
        
        self._update_speeds()

    def _update_speeds(self):
        self._frames['x'].append(self.current_data['x'])
        self._frames['y'].append(self.current_data['y'])
        self._frames['theta'].append(self.current_data['orientation'])

        self.vx = speed(self._frames['x'], self.game.vision._fps)
        self.vy = speed(self._frames['y'], self.game.vision._fps)
        self.vtheta = speed(self._frames['theta'], self.game.vision._fps)

        # print(self.get_name(), '= speeds: vx: {:.4f} m/s :: vy: {:.4f} m/s :: vt: {:.2f} RAD/s'.format(self.vx, self.vy, self.vtheta))


    def decide(self):
        # mocado, for a while :)
        power_left, power_right = 40, 40

        return self._get_command(power_left, power_right)


    def _get_command(self, pl, pr):
        return {
            'robot_id': self.robot_id,
            'wheel_left': pl,
            'wheel_right': pr,
            'color': self.team_color
        }
