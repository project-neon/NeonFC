from collections import deque
import copy

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
        self.x, self.y = 0, 0

    def get_name(self):
        return 'BALL'

    def get_speed(self):
        return (self.vx**2 + self.vy**2)**.5

    def update(self, frame):
        self.current_data = frame.get('ball')
        self._update_speeds()

    def pos_next(self, fps=10):
        ball_next = copy.copy(self)
        ball_next.x += ball_next.vx * 10 * self.game.vision._fps
        ball_next.y += ball_next.vy * 10 * self.game.vision._fps
        return ball_next

    def _update_speeds(self):
        self._frames['x'].append(self.current_data['x'])
        self._frames['y'].append(self.current_data['y'])

        self.x = self.current_data['x']
        self.y = self.current_data['y']

        self.vx = speed(self._frames['x'], self.game.vision._fps)
        self.vy = speed(self._frames['y'], self.game.vision._fps)