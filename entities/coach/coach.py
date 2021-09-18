from abc import ABC, abstractmethod
from os import replace

class BaseCoach(ABC):
    NAME = "BASE_COACH"
    def __init__(self, match):
        self.match = match

    @abstractmethod
    def decide (self):
        raise NotImplementedError("Coach needs decide implementation!")

    def get_positions(self, foul, team_color, foul_color):
        return None