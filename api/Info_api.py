
class Info_Api():
    def __init__(self, match, robots, coach, ball)  :
        
        self.match = match
        self.robots = robots
        self.coach = coach
        self.ball = ball

        self.data = {}


    def organize_send(self):

        data_send = dict({
            'COACH_NAME' :  self.match.coach_name,
            'TEAM_COLOR' :  self.match.team_color,
            'CATEGORY' :    self.match.category,
            'TEAM_ROBOTS_POS' : [{f"{robot.robot_id}": (robot.x, robot.y, robot.theta)} for robot in self.robots],
            'OPPOSITE_ROBOTS_POS' : [{f"{robot.robot_id}": (robot.x, robot.y, robot.theta)} for robot in self.match.opposites],
            'BALL_POS' : (self.ball.x, self.ball.y),
            'GAME_STATUS' : self.match.game_status,
            'TEAM_SIDE' : self.match.team_side
        })

        self.save_data(data_send)

        return data_send


    def update_recv(self,info_recv):

        self.match.update(**info_recv)

        for robot in self.robots:
            self.robot.update(**info_recv)

        self.save_data(info_recv)
        

    def save_data(self,info_save):

        for i in info_save:
            self.data.update({i: info_save[i]})

        