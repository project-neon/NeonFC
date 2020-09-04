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


class Ball(object):

    def __init__(self, game):
        self.game = game
        self.current_data = []

        self._frames = {
            'x': deque(maxlen=10),
            'y': deque(maxlen=10)
        }

        self.vx, self.vy = 0, 0

    def get_name(self):
        return 'BALL'

    def update(self, frame):
        self.current_data = frame.get('ball')
        self._update_speeds()


    def _update_speeds(self):
        self._frames['x'].append(self.current_data['x'])
        self._frames['y'].append(self.current_data['y'])

        self.x = self.current_data['x']
        self.y = self.current_data['y']

        self.vx = speed(self._frames['x'], self.game.vision._fps)
        self.vy = speed(self._frames['y'], self.game.vision._fps)