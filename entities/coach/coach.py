  
from abc import ABC, abstractmethod

class BaseCoach(ABC):
    NAME = "BASE_COACH"
    def __init__(self, match):
        self.match = match

    @abstractmethod
    def decide (self):
        raise NotImplementedError("Coach needs decide implementation!")

    def get_positions(self, foul, team_color):
        return None