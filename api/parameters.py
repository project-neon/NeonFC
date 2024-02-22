

class Parameter:
    def __init__(self, parameters = {"kp":1, "ki":0, "kd":0, "kw":3.5, "rm":0.44, "vm" : 0.5}):
        self.parameters = parameters

        self.kp = self.parameters.get("kp")
        self.ki = self.parameters.get("ki")
        self.kd = self.parameters.get("kd")
        self.kw = self.parameters.get("kw")
        self.rm = self.parameters.get("rm")
        self.vm = self.parameters.get("vm")
        
    def update_information(self, **kwargs): #Function to update values recieved in api
        for key, value in kwargs.items():
            if hasattr(self, key.lower()):
                setattr(self, key.lower(), value)
        
        self.kp = self.parameters.get("kp")
        self.ki = self.parameters.get("ki")
        self.kd = self.parameters.get("kd")
        self.kw = self.parameters.get("kw")
        self.rm = self.parameters.get("rm")
        self.vm = self.parameters.get("vm")
        
    def __add__(self, value):
        return self.value + value
        
    def __mul__(self, value):
        return self.value * value

    def __pow__(self, power):
        return self.value ** power