import entities 

class InfoApi():
    parameter_list = []

    def __init__(self, match, robots, opposites, coach, ball, parameters)  :
        
        self.match = match
        self.robots = robots
        self.opposites = opposites
        self.coach = coach
        self.ball = ball
        self.parameters = parameters

        self.coach_list = []
        for coach_name, entitie in entities.coach.COACHES.items(): 
            self.coach_list.append(coach_name)

        self.data = {}

    def organize_send(self):
        data_send = {
        'MATCH':
            {'COACH_NAME' :  self.match.coach_name,
             'COACH_LIST' : self.coach_list 
            },
         'TEAM_ROBOTS':
            {'ROBOT_POS' : {f"{robot.robot_id}": (robot.x, robot.y, robot.theta) for robot in self.robots},
             'STRATEGY' : {f"{robot.robot_id}": robot.strategy.name for robot in self.robots}
            },
         'OPPOSITE_ROBOTS':
            {'ROBOT_POS' : {f"{robot.robot_id}": (robot.x, robot.y, robot.theta) for robot in self.opposites}
            },
         'BALL':
            {'BALL_POS' : (self.ball.x, self.ball.y)
            }
        }

        self.save_data(data_send)

        return data_send


    def update_recv(self,info_recv):
        self.match.update_information(info_recv['MATCH'])


        for parameter in self.parameter_list:
            parameter.update(info_recv['PARAMETERS'])

        self.save_data(info_recv)
        

    def save_data(self,info_save):

        for i in info_save:
            self.data.update({i: info_save[i]})

        