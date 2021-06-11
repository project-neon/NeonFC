import json

class Field():
    def __init__(self, team_color, num_robots=3, coach_name=None, category="3v3"):
        self.game_mode = category
        self.source = json.loads(open('fields.json', 'r').read())
        self.field = self.source.get(self.game_mode)

    def get_field(self):
        fieldSize = self.field.get('fieldSize', 0)
        return fieldSize
    
    def get_small_area(self):
        #depois essa função deve retornar [x1, y1, w, h]
        small_area = self.field.get('sAreaSize', 0)
        return small_area

    def get_quad_ref(self, quad):
        quadrants = self.field.get('quadRef')
        quaDimen = quadrants.get(f'q{quad}')
        return quaDimen

    def get_fk_pos(self, side):
        freekicks = self.field.get("freeKick")
        fk_pos = freekicks.get(side)
        return fk_pos
