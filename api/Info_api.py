class InfoApi():
    def __init__(self, match, robots, opposites, coach, ball, parameters)  :
        
        self.match = match
        self.robots = robots
        self.opposites = opposites
        self.coach = coach
        self.ball = ball
        self.parameters = parameters

        self.data = {}

    def organize_send(self):
        data_send = dict({
            'COACH_NAME' :  self.match.coach_name,
            'TEAM_ROBOTS_POS' : [{f"{robot.robot_id}": (robot.x, robot.y, robot.theta)} for robot in self.robots],
            'OPPOSITE_ROBOTS_POS' : [{f"{robot.robot_id}": (robot.x, robot.y, robot.theta)} for robot in self.match.opposites],
            'ROBOTS_STRATEGY' : [{f"{robot.robot_id}": robot.strategy.name} for robot in self.robots],
            'BALL_POS' : (self.ball.x, self.ball.y)
        })

        self.save_data(data_send)

        return data_send


    def update_recv(self,info_recv):
        self.match.update_information(**info_recv)
        
        for parameter in self.parameters:
            parameter.update(**info_recv)

        self.save_data(info_recv)
        

    def save_data(self,info_save):

        for i in info_save:
            self.data.update({i: info_save[i]})

        