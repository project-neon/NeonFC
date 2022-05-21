from algorithms.potential_fields.fields import PotentialField
from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
import math
import algorithms

# class MainAttacker(DebugPotentialFieldStrategy):
class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=UniController)

    def start(self, robot=None):
        super().start(robot=robot)

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def AvoidBound(self):
        """
        returns the wheel velocity (vl, vr) of the robot close to a wall in order to avoid it getting stuck on the wall
        """

        field_w = self.match.game.field.get_dimensions()[0]
        field_h = self.match.game.field.get_dimensions()[1]

        angle_bound = 60 # graus
        distance_bound = 0.09 # m

        corner_angle = 75
        corner_distance = 0.12
        
        dx = self.match.ball.x - self.robot.x
        dy = self.match.ball.y - self.robot.y

        theta_d = int((180 / math.pi) * math.atan2(dy, dx))
        if dx == 0 and dy == 0:
            theta_d = 90
        
        theta = self.robot.theta * (180 / math.pi) # graus
        theta_e = theta_d - theta
        
        while theta > 180:
            theta -= 360
        while theta < -180:
            theta += 360
        
        while theta_e > 180:
            theta_e -= 360
        while theta_e < -180:
            theta_e += 360
        
        # for top and bottom walls
        if self.robot.y > (field_h - distance_bound): # top
            if self.robot.x < (corner_distance) or self.robot.x > (field_w - corner_distance):
                angle_bound = corner_angle
            if theta > (-90 - angle_bound) and theta < (-90 + angle_bound):
                # top: case T1
                vr = int(14 + (19 / 90) * theta_e)
                vl = int(14 - (19 / 90) * theta_e)
                return vl, vr
            elif theta > (90 - angle_bound) and theta < (90 + angle_bound):
                # top: case T2
                vr = int(-14 + (19 / 90) * theta_e)
                vl = int(-14 - (19 / 90) * theta_e)
                return vl, vr
        elif self.robot.y < distance_bound: # bottom
            if self.robot.x < (corner_distance) or self.robot.x > (field_w - corner_distance):
                angle_bound = corner_angle
            if theta > (-90 - angle_bound) and theta < (-90 + angle_bound):
                # bottom: case B1
                vr = int(-14 + (19 / 90) * theta_e)
                vl = int(-14 - (19 / 90) * theta_e)
                return vl, vr
            elif theta > (90 - angle_bound) and theta < (90 + angle_bound):
                # bottom: case B2
                vr = int(14 + (19 / 90) * theta_e)
                vl = int(14 - (19 / 90) * theta_e)
                return vl, vr
        
        m_theta = abs(theta)
        # for left-side wall
        if self.robot.x < distance_bound:
            if self.robot.y < (corner_distance) or self.robot.y > (field_h - corner_distance):
                angle_bound = corner_angle
            if m_theta < angle_bound and self.robot.y > 0.85:
                # left wall: case L1
                vr = int(14 + (19 / 90) * theta_e)
                vl = int(14 - (19 / 90) * theta_e)
                return vl, vr
            elif m_theta > (180 - angle_bound) and self.robot.y > 0.85:
                # left wall: case L2
                vr = int(-14 + (19 / 90) * theta_e)
                vl = int(-14 - (19 / 90) * theta_e)
                return vl, vr
            elif m_theta < angle_bound and self.robot.y < 0.45:
                # left wall: case L3
                vr = int(14 + (19 / 90) * theta_e)
                vl = int(14 - (19 / 90) * theta_e)
                return vl, vr
            elif m_theta > (180 - angle_bound) and self.robot.y < 0.45:
                # left wall: case L4
                vr = int(-14 + (19 / 90) * theta_e)
                vl = int(-14 - (19 / 90) * theta_e)
                return vl, vr
        
        # right-side wall
        if self.robot.x > (field_w - distance_bound):
            if self.robot.y < (corner_distance) or self.robot.y > (field_h - corner_distance):
                angle_bound = corner_angle
            if m_theta < angle_bound and self.robot.y > 0.85:
                # right wall: case R1
                vr = int(-14 + (19 / 90) * theta_e)
                vl = int(-14 - (19 / 90) * theta_e)
                return vl, vr
            elif m_theta > (180 - angle_bound) and self.robot.y > 0.85:
                # right wall: case R2
                vr = int(14 + (19 / 90) * theta_e)
                vl = int(14 - (19 / 90) * theta_e)
                return vl, vr
            elif m_theta < angle_bound and self.robot.y < 0.45:
                # right wall: case R3
                vr = int(-14 + (19 / 90) * theta_e)
                vl = int(-14 - (19 / 90) * theta_e)
                return vl, vr
            elif m_theta > (180 - angle_bound) and self.robot.y < 0.45:
                # right wall: case R4
                vr = int(14 + (19 / 90) * theta_e)
                vl = int(14 - (19 / 90) * theta_e)
                return vl, vr
        
        return 0, 0

    def uvf(self):
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

    def decide(self, x=None, y=None):
        self.controller.set_flag(False, 0, 0)

        if x:
            self.robot.x = x
        if y:
            self.robot.y = y

        vl, vr = self.AvoidBound()
        if vl != 0 or vr != 0:
            self.controller.set_flag(True, vl, vr)
            return [vl, vr]

        return self.uvf()
        
        
