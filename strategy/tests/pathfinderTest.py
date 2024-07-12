from typing import Tuple

from controller import PID_control
from strategy.BaseStrategy import Strategy
import time

class PathfinderTest(Strategy):

    time_init = 0

    def __init__(self, match):
        super().__init__(
            match, 'pathfinderTest',
            controller=PID_control
        )
        self.time_init = time.time()
        self.field_limits = [0.75*2, 0.65*2]

    def get_play(self): return None

    def decide(self) -> tuple[float, float]:
        phaseTime = self.time_init % 60 #segundos pq Python é avançado demais pra usar milis
        #ret = tuple[0.0,0.0]
        if phaseTime < 10: return self.field_limits[0] * .25 , self.field_limits[1] * .75
        elif phaseTime < 20: return self.field_limits[0] * .25 , self.field_limits[1] * .25
        elif phaseTime < 30: return self.field_limits[0] * .75 , self.field_limits[1] * .25
        elif phaseTime < 40: return self.field_limits[0] * .75 , self.field_limits[1] * .75
        elif phaseTime < 50: return self.field_limits[0] * .5 , self.field_limits[1] * .5
        return 0.0, 0.0
