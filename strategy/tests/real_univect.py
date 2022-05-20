import math
import algorithms
import controller

from strategy.BaseStrategy import Strategy


class RealUnivectField(Strategy):
    def __init__(self, match, name="UVF2.0"):
        super().__init__(match, name, controller=controller.TwoSidesLQR)

    
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
        spiral_size = 0.027 # radius_spiral
        kr = 0.055 # segundo o paper: "If Kr becomes larger, the spiral becomes smoother."

        def dist_target_point(point_x, point_y):
            dist = math.sqrt((point_x - self.match.ball.x)**2 + (point_y - self.match.ball.y)**2)
            return dist


        def theta_targetx_point(point_x, point_y):
            theta = math.atan2((point_y - self.match.ball.y), (point_x - self.match.ball.x))
            return theta
        
        #cw = clock wise
        #ccw = counter clock wise
        # dist between ball and robot > radius
        def left_point_outside_ccw(p, radius_e=spiral_size):
            phi = theta_targetx_point(self.robot.x, self.robot.y) + (math.pi/2)*(2-((radius_e+kr)/(p+kr)))
            x = math.cos(phi)
            y = math.sin(phi)
            return [x, y]


        def right_point_outside_cw(p, radius_e=spiral_size):
            phi = theta_targetx_point(self.robot.x, self.robot.y) - (math.pi/2)*(2-((radius_e+kr)/(p+kr)))
            x = math.cos(phi)
            y = math.sin(phi)
            return [x, y]


        def left_point_inside_cw(p ,radius_e=spiral_size):
            phi = theta_targetx_point(self.robot.x, self.robot.y) + (math.pi/2)*math.sqrt(p/radius_e)
            x = math.cos(phi)
            y = math.sin(phi)
            return [x, y]
            
            
        def right_point_inside_ccw(p , radius_e=spiral_size):
            phi = theta_targetx_point(self.robot.x, self.robot.y) - (math.pi/2)*math.sqrt(p/radius_e)
            x = math.cos(phi)
            y = math.sin(phi)
            return [x, y]

        dist_ball_robot = dist_target_point(self.robot.x, self.robot.y)

        yl = left_point_inside_cw( dist_ball_robot, spiral_size)[1] + spiral_size
        yr = right_point_inside_ccw( dist_ball_robot, spiral_size)[1] - spiral_size
        
        pl_x = left_point_inside_cw( dist_ball_robot, spiral_size)[0]
        pl_y = left_point_inside_cw( dist_ball_robot, spiral_size)[1] - spiral_size
        
        pr_x = right_point_inside_ccw( dist_ball_robot, spiral_size)[0]
        pr_y = right_point_inside_ccw( dist_ball_robot, spiral_size)[1] + spiral_size


        def move_to_goal_between_points(radius_e=spiral_size):
            pos = (((yl * right_point_inside_ccw(dist_target_point(pl_x, pl_y), radius_e)[1]) + 
                           (yr * left_point_inside_cw(dist_target_point(pr_x, pr_y), radius_e)[1]))/(2*radius_e))
            
            future_x = pos[0]
            future_y = pos[1]

            return [future_x, future_y]


        def move_to_goal_under_point(radius_e=spiral_size):
            pos = right_point_outside_cw(dist_target_point(pl_x, pl_y), radius_e)
            
            future_x = pos[0]
            future_y = pos[1]

            return [future_x, future_y]
            

        def move_to_goal_above_point(radius_e=spiral_size):
            pos = left_point_outside_ccw(dist_target_point(pr_x, pr_y), radius_e)

            future_x = pos[0]
            future_y = pos[1]

            return [future_x, future_y]

        if self.robot.y > spiral_size:
            position = move_to_goal_above_point()

        elif self.robot.y < spiral_size:
            position = move_to_goal_under_point()

        else:
            position = move_to_goal_between_points()

        future_x = position[0] 
        future_y = position[1]

        return (future_x, future_y)