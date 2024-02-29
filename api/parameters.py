
class Parameter:
    def __init__(self, default, address):

        self.value = default
        self.addr = address
        
    def update(self, **kwargs):
        for key, value in kwargs.items():
            if key == 'PARAMETERS':
                self.value = value[self.addr[0]][self.addr[1]]
        
    def __add__(self, value):
        return self.value + value
        
    def __mul__(self, value):
        return self.value * value
       
    def __pow__(self, power):
        return self.value ** power