from socket import *
import json

class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.
    source: https://refactoring.guru/design-patterns/singleton/python/example
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class Api(metaclass=SingletonMeta):
    def __init__(self, address, port):
        
        self.address = address
        self.port = port

    # Initiate socket connection
    def start(self):
        self.obj_socket = socket(AF_INET, SOCK_DGRAM)

    # Sends dict game data to socket listener
    def send_data(self, obj):
        data_dict = dict({
            'COACH_NAME' :  obj.coach_name,
            'TEAM_COLOR' :  obj.team_color,
            'CATEGORY' :    obj.category,
            'TEAM_ROBOTS_POS' : [{f"{robot.robot_id}": (robot.x, robot.y, robot.theta)} for robot in obj.robots],
            'OPPOSITE_ROBOTS_POS' : [{f"{robot.robot_id}": (robot.x, robot.y, robot.theta)} for robot in obj.opposites],
            'BALL_POS' : (obj.ball.x, obj.ball.y),
            'GAME_STATUS' : obj.game_status,
            'TEAM_SIDE' : obj.team_side
        })
        msg = json.dumps(data_dict)
        self.obj_socket.sendto(msg.encode(), (self.address, self.port))
    
    def send_custom_data(self, data):
         msg = json.dumps(data)
         self.obj_socket.sendto(msg.encode(), (self.address, self.port))
