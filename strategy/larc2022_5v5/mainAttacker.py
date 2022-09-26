from collections import deque
import math
import threading
from algorithms.potential_fields.fields import PotentialField, TangentialField
from commons import math as nfc_math
import controller
from strategy.BaseStrategy import Strategy
from algorithms.astar import astart_voronoi

def aim_projection_ball(strategy):
    m = strategy.match
    b = strategy.match.ball

    ball = [b.x, b.y]
    goal_pos = [
        m.game.field.get_dimensions()[0],
        m.game.field.get_dimensions()[1]/2
    ]

    dir_to_goal_vector = [
        goal_pos[0] - ball[0], 
        goal_pos[1] - ball[1]
    ]
    angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

    return ball[0] - 0.2* math.cos(angle), ball[1] - 0.2 * math.sin(angle)

class AstarPlanning(threading.Thread):
    def __init__(self, match, robot):
        super().__init__()
        self.match = match
        self.robot = robot

        self.iteration = 0

        self.path = []

        

    def run(self):
        while True:
            if self.robot.strategy:
                self.path = astart_voronoi.voronoi_astar(self.robot.strategy, self.match, aim_projection_ball)
                self.iteration += 1

                if self.path:
                    self.path = self.path[1:]

class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=controller.PID_control)

        self.planning = None
        self.path = []

        self.actual_iteration = -1

        self.is_on_attack = False



    def start(self, robot=None):
        super().start(robot=robot)

        self.planning = AstarPlanning(self.match, robot)
        self.planning.start()

        self.seek = PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        def get_aim_point(m):
            field_w, field_h = m.game.field.get_dimensions()
            aim_point_x = field_w
            if field_h/2 - 0.2 < m.ball.y < field_h/2 + 0.2:
                aim_point_y = m.ball.y
            else:
                aim_point_y = field_h/2
            return aim_point_x, aim_point_y

        def shifted_target_left(m, radius_2=0.075):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)

            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05

            pos_x = (
                m.ball.x -
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2)*radius_2
            )

            pos_y = (
                m.ball.y -
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2)*radius_2
            )

            return [pos_x, pos_y]

        def shifted_target_right(m, radius=0.075):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)

            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05

            pos_x = (
                m.ball.x +
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2) * radius
            )

            pos_y = (
                m.ball.y +
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))+ math.pi/2) * radius
            )

            return [pos_x, pos_y]
        
        def uvf_mean_contributtion_left(m, radius=0.075, robot=self.robot, speed=lambda _: 0.5):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)

            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05
            
            target = [
                m.ball.x,
                m.ball.y
            ]

            target2 = [
                m.ball.x +
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))) * 0.025,
                m.ball.y +
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x- m.ball.x))) * 0.025
            ]

            dist = nfc_math.distance_to_line(pos[0], pos[1], target[0], target[1], target2[0], target2[1])
            if abs(dist) > radius and dist < 0:
                return 0
            dist = 0.5 * max(0, min(dist, radius))/ (radius)
            return speed(m) * (dist + 0.5)

        def uvf_mean_contributtion_right(m, radius=0.075, robot=self.robot, speed=lambda _: 0.5):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_x, aim_point_y = get_aim_point(m)
            
            if m.ball.y > field_w - 0.075:
                aim_point_y = field_w - 0.05
            if m.ball.y < 0.075:
                aim_point_y = 0.05
            
            target = [
                m.ball.x,
                m.ball.y
            ]

            target2 = [
                m.ball.x +
                math.cos(math.atan2((aim_point_y-m.ball.y), (aim_point_x - m.ball.x))) * 0.025,
                m.ball.y +
                math.sin(math.atan2((aim_point_y-m.ball.y), (aim_point_x- m.ball.x))) * 0.025
            ]

            dist = -nfc_math.distance_to_line(pos[0], pos[1], target[0], target[1], target2[0], target2[1])
            if abs(dist) > radius and dist < 0:
                return 0
            dist = 0.5 * max(0, min(dist, radius))/ (radius )
            return speed(m) * (dist + 0.5)

        self.seek.add_field(
            TangentialField(
                self.match,
                target=shifted_target_left,                                                                                                                                                                                                                                                                                                                                          
                radius = 0.075,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_left
            )
        )

        self.seek.add_field(
            TangentialField(
                self.match,
                target=shifted_target_right,                                                                                                                                                                                                                                                                                                                                          
                radius = 0.075,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_right
            )
        )


    def next_point(self):
        if self.planning.iteration > self.actual_iteration:
            self.path = deque(self.planning.path)
            self.actual_iteration = self.planning.iteration
        
        if len(self.path) == 0:
            return self.robot.x, self.robot.y
        
        point = self.path[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        if math.sqrt(dx**2 + dy**2) < 0.05:
            self.path.rotate(-1)

        
        return self.path[0]

    def next_to(self, p1, p2, err=0.05):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]

        if math.sqrt(dx**2 + dy**2) < err:
            return True

        return False

    def decide(self):
        
        robot_pos = [self.robot.x, self.robot.y]
        aim_pos = aim_projection_ball(self)

        if not self.next_to(robot_pos, aim_pos, 0.30):
            self.is_on_attack = False
        
        if self.next_to(robot_pos, aim_pos, 0.15) or self.is_on_attack == True:
            self.is_on_attack = True
            print("ball")
            vector = self.seek.compute([self.robot.x, self.robot.y])
            angle = math.atan2(vector[1], vector[0])

            return self.robot.x + math.cos(angle) * 0.1 , self.robot.y  + math.sin(angle) * 0.1

        if len(self.planning.path):
            print("astar")
            return self.next_point()
        
        return 0, 0
