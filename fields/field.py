import json

class Field():
    def __init__(self, category="3v3"):
        self.game_mode = category
        self.source = json.loads(open('fields.json', 'r').read())
        self.field = self.source.get(self.game_mode)

    #the dimensios are defined in fields.json and all of them are in meters
    def get_dimensions(self):
        #get field dimensions and return width and height 
        field_size = self.field.get('field_size', 0)
        return field_size
    
    def get_small_area(self, team):
        #this function return the coordinates (x1, y1, width, height) of the small area based on arg team, 
        #that recieves team color
        areas = self.field.get('small_area', 0)
        small_area = areas.get(team)
        return small_area

    def get_quadrant_position(self, quad):
        #quadrants are definied as the quadrants from trigonometry, therefore, q1 is in upper right corner
        #and they are in counterclockwise.
        #return quadrant positions (x, y) based on arg quad, that recieves an integer number.
        quadrants = self.field.get('quadrant_positions')
        quad_dimen = quadrants.get(f'q{quad}')
        return quad_dimen

    def get_free_kick_position(self, side):
        #return free kick position (x, y) based on arg side, that recieves the field side
        free_kicks = self.field.get("free_kick")
        fk_pos = free_kicks.get(side)
        return fk_pos
