from strategy.BaseStrategy import Strategy
from algorithms.potential_fields.fields import PotentialField
import math
import numpy as np
from controller.PID_control import PID_control

class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=PID_control)

    def __init__(self, match, name):
        controller_args = {
            'max_speed':2,
            'max_angular':8400, 
            'krho':10, 
            'kp':200,
            'ki': 0,
            'kd': 1,
        }
        super().__init__(match, name, PID_control, controller_args)

    def start(self, robot=None):
        super().start(robot=robot)
 
    
    def decide(self):
        
        return [0,0]
    
   

    
