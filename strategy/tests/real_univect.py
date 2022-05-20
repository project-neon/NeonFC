import math
import controller

from strategy.BaseStrategy import Strategy


class RealUnivectField(Strategy):
    def __init__(self, match, name="UVF2.0"):
        super().__init__(match, name, controller=controller.UniController)

    
    def start(self, robot=None):
        super().start(robot=robot)
            
    
    def decide(self, x=None, y=None):
        if x:
            self.robot.x = x
        if y:
            self.robot.y = y

        """
        Precisa tunar essas constantes
        """
        spiral_size = 0.09 # radius_spiral
        kr = 0.8 # Segundo o paper: "If Kr becomes larger, the spiral becomes smoother." OBS.: começa 
                 # a ficar bom na casa do 0.6 pra cima.

        ball = self.match.ball
        robot = self.robot


        def _dist_target_point(point_x, point_y, target_x, target_y):
            dist = math.sqrt((point_x - target_x)**2 + (point_y - target_y)**2)
            
            return dist


        def _theta_targetx_point(point_x, point_y, target_x, target_y):
            theta = math.atan2((point_y - target_y), (point_x - target_x))
            
            return theta
        

        # cw = clock wise
        # ccw = counter clock wise
        # if dist between ball and robot > radius
        def left_point_outside_ccw(p, radius_e=spiral_size):
            theta = _theta_targetx_point(robot.x, robot.y, ball.x, ball.y)
            phi = theta + (math.pi/2)*(2-((radius_e+kr)/(p+kr)))
            
            x = math.cos(phi) 
            y = math.sin(phi)
            
            return [x, y]


        def right_point_outside_cw(p, radius_e=spiral_size):
            theta = _theta_targetx_point(robot.x, robot.y, ball.x, ball.y)
            phi = theta - (math.pi/2)*(2-((radius_e+kr)/(p+kr)))
            
            x = math.cos(phi) 
            y = math.sin(phi)
            
            return [x, y]

        # if 0 < dist between ball and radius < radius
        def left_point_inside_cw(p ,radius_e=spiral_size):
            theta = _theta_targetx_point(robot.x, robot.y, ball.x, ball.y)
            phi = theta + (math.pi/2)*math.sqrt(p/radius_e)
            
            x = math.cos(phi)
            y = math.sin(phi)
            
            return [x, y]
            
            
        def right_point_inside_ccw(p , radius_e=spiral_size):
            theta = _theta_targetx_point(robot.x, robot.y, ball.x, ball.y)
            phi = theta - (math.pi/2)*math.sqrt(p/radius_e)
            
            x = math.cos(phi)
            y = math.sin(phi)
            
            return [x, y]


        # to calculate future robot coord
        dist_ball_robot = _dist_target_point(robot.x, robot.y, ball.x, ball.y)

        yl = left_point_inside_cw( dist_ball_robot, spiral_size)[1] + spiral_size
        yr = right_point_inside_ccw( dist_ball_robot, spiral_size)[1] - spiral_size
        
        pl_x = left_point_inside_cw( dist_ball_robot, spiral_size)[0]
        pl_y = left_point_inside_cw( dist_ball_robot, spiral_size)[1] - spiral_size
        
        pr_x = right_point_inside_ccw( dist_ball_robot, spiral_size)[0]
        pr_y = right_point_inside_ccw( dist_ball_robot, spiral_size)[1] + spiral_size


        # if robot.y is inside the spiral radius
        def move_to_goal_between_points(radius_e=spiral_size):
            pos = (((yl * right_point_inside_ccw(_dist_target_point(pl_x, pl_y, ball.x, ball.y), radius_e)[1]) + 
                    (yr * left_point_inside_cw(_dist_target_point(pr_x, pr_y, ball.x, ball.y), radius_e)[1]))/(2*radius_e))
            
            future_x = pos[0]
            future_y = pos[1]

            return [future_x, future_y]


        # if robot.y is under the spiral radius
        def move_to_goal_under_point(radius_e=spiral_size):
            pos = right_point_outside_cw(_dist_target_point(pl_x, pl_y, ball.x, ball.y), radius_e)
            
            future_x = pos[0]
            future_y = pos[1]

            return [future_x, future_y]
            

        # if robot.y is above the spiral radius
        def move_to_goal_above_point(radius_e=spiral_size):
            pos = left_point_outside_ccw(_dist_target_point(pr_x, pr_y, ball.x, ball.y), radius_e)

            future_x = pos[0]
            future_y = pos[1]

            return [future_x, future_y]

        if robot.y > ball.y + spiral_size:
            position = move_to_goal_above_point()

        else:
            position = move_to_goal_under_point()

        """else:
            position = move_to_goal_between_points()"""

        future_x = position[0] 
        future_y = position[1]

        return (future_x, future_y)