import json

class Fields():
    def __init__(self, game_mode):
        self.game_mode = game_mode
        self.field = json.loads(open("{}_field.json".format(game_mode), 'r').read())
    
    def getField(self):
    	fieldSize = self.field.get("fieldSize", 0)
    	return fieldSize

f = Fields("3v3")
print(f.field, f.game_mode, f.getField())
