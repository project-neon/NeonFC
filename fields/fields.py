import json

class Field():
    def __init__(self, team_color, num_robots=3, coach_name=None, category="3v3"):
        self.game_mode = category
        self.source = json.loads(open('fields.json', 'r').read())
        self.field = self.source.get(self.game_mode)

    def get_field(self):
        fieldSize = self.field.get('fieldSize', 0)
        w = fieldSize.get('w')
        h = fieldSize.get('h')
        return (w, h)
    
    def get_small_area(self):
        small_area = self.field.get('sAreaSize', 0)
        w = self.field['sAreaSize'].get('w', 0)
        h = small_area.get('h', 0)
        return (w, h)

    def get_quad_ref(self, quad):
        quadrants = self.field.get('quadRef')
        quaDimen = quadrants.get(f'q{quad}')
        x, y = quaDimen
        return (x, y)

    def get_fk_pos(self, side):
        freekicks = self.field.get("freeKick")
        fk_pos = freekicks.get(side)
        x, y = fk_pos
        return (x, y)
