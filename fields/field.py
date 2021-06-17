import json

class Field():
    def __init__(self, category="3v3"):
        self.game_mode = category
        self.source = json.loads(open('fields.json', 'r').read())
        self.field = self.source.get(self.game_mode)

    def get_dimensions(self):
        #get field dimensions
        field_size = self.field.get('field_size', 0)
        return field_size
    
    def get_small_area(self, team):
        #this function return the coordinates (x1, y1, w, h) of the small area
        areas = self.field.get('small_area', 0)
        small_area = areas.get(team)
        return small_area

    def get_quadrant_position(self, quad):
        #return quadrant positions (x, y)
        quadrants = self.field.get('quadrant_positions')
        quad_dimen = quadrants.get(f'q{quad}')
        return quad_dimen

    def get_free_kick_position(self, side):
        free_kicks = self.field.get("free_kick")
        fk_pos = free_kicks.get(side)
        return fk_pos
