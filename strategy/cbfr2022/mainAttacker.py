from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from controller.uni_controller import UniController
import math
import algorithms

# class MainAttacker(DebugPotentialFieldStrategy):
class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=UniController)
    
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
    
    def start(self, robot=None):
        super().start(robot=robot)

        self.point = algorithms.fields.PotentialField(
            self.match,
            name="{}|PointBehaviour".format(self.__class__)
        )

        self.point.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y), #(0.75, 1.65),
                radius = .075,
                decay = lambda x: x,
                multiplier = 1
            )
        )

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def decide(self, x=None, y=None):
        self.controller.set_flag(False, 0, 0)

        if x:
            self.robot.x = x
        if y:
            self.robot.y = y

        behaviour = None

        behaviour = self.point

        vl, vr = self.AvoidBound()
        if vl != 0 or vr != 0:
            self.controller.set_flag(True, vl, vr)
            return [vl, vr]

        return behaviour.compute([self.robot.x, self.robot.y])
        
        
