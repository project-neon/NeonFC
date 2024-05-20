from api import InfoApi

class Parameter:
    def __init__(self, default, address):
        InfoApi.parameter_list.append(self)
        self.value = default
        self.addr = address
        
    def update(self, info):
        self.value = info[self.addr.upper()]
        
    def __add__(self, value):
        return self.value + value
        
    def __mul__(self, value):
        return self.value * value
    
    def __rmul__(self, value):
        return self.value * value

    def __pow__(self, power):
        return self.value ** power
    
    def __lt__(self, value): 
        return self.value < value
    
    def __gt__(self, value): 
        return self.value > value
    
    def __le__(self, value): 
        return self.value <= value

    def __ge__(self, value): 
          return self.value >= value