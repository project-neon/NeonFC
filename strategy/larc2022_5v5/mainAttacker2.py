import math
import random
import threading

from scipy.spatial import Voronoi

from collections import deque
from algorithms.astar.fieldGraph import FieldGraph
from algorithms.potential_fields import fields
from controller.simple_LQR import TwoSidesLQR

from strategy.BaseStrategy import Strategy

from controller import PID_control

from commons import math as nfc_math

from algorithms.astar.astart_voronoi import voronoi_astar
from algorithms.limit_cycle import LimitCycle, Point
from strategy.utils.player_playbook import OnAttackerPushTrigger, OnDefensiveTransitionTrigger, OnNextTo, PlayerPlay, PlayerPlaybook

def aim_projection_ball(strategy,):
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

class AstarPlanning(PlayerPlay):
    def __init__(self, match, robot):
        threading.Thread.__init__(self)
        super().__init__(
            match, 
            robot
        )

        self.actual_iteration = 0
        self.next_iterarion = 0

        self.path = []
        self.next_path = []

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2.75}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def get_name(self):
        return f"<{self.robot.get_name()} Voronoi Astar Planning>"

    def run(self):
        if self.robot.strategy:
            if self.next_iterarion % 60 * 15 == 0:
                print(f"VAI {self.next_iterarion}")
                self.next_path = voronoi_astar(
                    self.robot.strategy, self.match, aim_projection_ball
                )
                if self.next_path:
                    self.next_path = self.next_path[1:]

            self.next_iterarion += 1

 

    def next_point(self):
        self.run()

        if self.next_iterarion > self.actual_iteration:
            self.path = deque(self.next_path)
            self.actual_iteration = self.next_iterarion
        
        if len(self.path) == 0:
            return self.robot.x, self.robot.y
        
        point = self.path[0]
        dx = point[0] - self.robot.x
        dy = point[1] - self.robot.y

        print(math.sqrt(dx**2 + dy**2))
        if math.sqrt(dx**2 + dy**2) < 0.2:
            print("next point!")
            self.path.rotate(-1)
        
        return self.path[0]
    
    def update(self):
        res = self.next_point()
        if len(res):
            return res
        else:
            return 0, 0

class LimitCyclePlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)
        self.match = match
        self.robot = robot

        self.limit_cycle = LimitCycle(
            self.match, 
            target_is_ball=True
        )

        self.desired_point = None

    def start_up(self):
            super().start_up()
            controller = PID_control
            controller_kwargs = {'max_speed': 2}
            self.robot.strategy.controller = controller(self.robot, **controller_kwargs)

    def get_name(self):
        return f"<{self.robot.get_name()} LimitCycle Planning>"

    def run(self):
        desired = 0, 0
        if self.robot.strategy:
            robot = Point(self.robot.x, self.robot.y)
            target = Point(self.match.ball.x, self.match.ball.y)

            if not (0 <= target.x <= 1.5 * 2) and not (0 <= target.y <= 1.3 * 2):
                target = Point(self.limit_cycle.target.x, self.limit_cycle.target.y)

            self.limit_cycle.update(robot, target, [])

            desired = self.limit_cycle.compute()


        return desired
    
    def update(self):
        res = self.run()
        return res


class WaitPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

        self.sa_x, self.sa_y, self.sa_w, self.sa_h = self.match.game.field.get_small_area("defensive")
        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def start_up(self):
        super().start_up()
        controller = PID_control
        self.robot.strategy.controller = controller(self.robot)

    def update(self):
        m = self.match
        if m.ball.y > self.sa_y + self.sa_h and m.ball.y > self.robot.y:
            x = 0.0375
            y = self.sa_y + self.sa_h + 0.0375
        elif m.ball.y < self.sa_y and m.ball.y < self.robot.y:
            x = 0.0375
            y = self.sa_y - 0.0375
        else:
            if m.ball.y > self.field_h/2:
                x = self.sa_w - 0.09 
                y = self.field_h - 0.1
            else:
                x = self.sa_w - 0.07
                y = 0.1

        return x, y
                

class PushPotentialFieldPlanning(PlayerPlay):
    def __init__(self, match, robot):
        super().__init__(match, robot)

    def get_name(self):
        return f"<{self.robot.get_name()} Push Potential Planning>"

    def start_up(self):
            super().start_up()
            controller = TwoSidesLQR
            self.robot.strategy.controller = controller(self.robot, l=0.07)

    def update(self):
        return super().update()

    def start(self):
        self.seek = fields.PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        uvf_radius = 0.075 # 7.5 cm
        uvf_radius_2 = 0.075 # 7.5 cm

        tangential_speed = lambda m: max(.8, (m.ball.vx**2 + m.ball.vy**2)**.5 + .3 ) # 8 cm/s


        def shifted_target_left(m, radius_2=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2
            aim_point_x = field_h

            ball = [m.ball.x, m.ball.y]
            goal_pos = [
                m.game.field.get_dimensions()[0],
                m.game.field.get_dimensions()[1]/2
            ]

            dir_to_goal_vector = [
                goal_pos[0] - ball[0], 
                goal_pos[1] - ball[1]
            ]
            angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

            ball_x = ball[0] - 0.07 * math.cos(angle)
            ball_y = ball[1] - 0.07 * math.sin(angle)

            pos_x = (
                ball_x -
                math.cos(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2)*radius_2
            )

            pos_y = (
                ball_y -
                math.sin(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2)*radius_2
            )

            return [pos_x, pos_y]

        def shifted_target_right(m, radius=uvf_radius_2):
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2
            aim_point_x = field_h

            ball = [m.ball.x, m.ball.y]
            goal_pos = [
                m.game.field.get_dimensions()[0],
                m.game.field.get_dimensions()[1]/2
            ]

            dir_to_goal_vector = [
                goal_pos[0] - ball[0], 
                goal_pos[1] - ball[1]
            ]
            angle = math.atan2(dir_to_goal_vector[1], dir_to_goal_vector[0])

            ball_x = ball[0] - 0.07 * math.cos(angle)
            ball_y = ball[1] - 0.07 * math.sin(angle)

            pos_x = (
                ball_x +
                math.cos(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2) * radius
            )

            pos_y = (
                ball_y +
                math.sin(math.atan2((aim_point_y-ball_y), (aim_point_x - ball_x))+ math.pi/2) * radius
            )

            return [pos_x, pos_y]
        
        def uvf_mean_contributtion_left(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2

            aim_point_x = field_h
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

        def uvf_mean_contributtion_right(m, radius=uvf_radius_2, robot=self.robot, speed=tangential_speed):
            pos =  [robot.x, robot.y]
            field_h, field_w = m.game.field.get_dimensions()
            aim_point_y = field_w/2

            aim_point_x = field_h
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
            fields.TangentialField(
                self.match,
                target=shifted_target_left,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_left
            )
        )

        self.seek.add_field(
            fields.TangentialField(
                self.match,
                target=shifted_target_right,                                                                                                                                                                                                                                                                                                                                          
                radius = uvf_radius,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = uvf_mean_contributtion_right
            )
        )

    def update(self):
        robot_pos = [self.robot.x, self.robot.y]
        return self.seek.compute(robot_pos)

class MainAttacker(Strategy):
    def __init__(self, match, name="MainAttacker"):
        super().__init__(match, name, controller=TwoSidesLQR)

        self.planning = None
        self.path = []

        self.actual_iteration = -1

        self.is_on_attack = False

        self.playerbook = None



    def start(self, robot=None):
        super().start(robot=robot)

        # Criando Player Playbook: A maquina de estados do jogador
        self.playerbook = PlayerPlaybook(self.match.coach, self.robot)

        # Criando Path Planning baseado em Astar
        astar = AstarPlanning(self.match, self.robot)
        # astar.start()

        # Criando Path Planning baseado em Limit Cycle
        limitcycle = LimitCyclePlanning(self.match, self.robot)

        # Criando Potential Field
        push_potentialfield = PushPotentialFieldPlanning(self.match, self.robot)
        push_potentialfield.start()

        wait = WaitPlanning(self.match, self.robot)


        # Adiciona ambas plays no livro do jogador
        self.playerbook.add_play(astar)
        self.playerbook.add_play(limitcycle)
        self.playerbook.add_play(push_potentialfield)

        self.playerbook.add_play(wait)

        # # Transicao para caso esteja perto da bola ( < 10 cm)
        next_to_ball_transition = OnNextTo(
            self.robot, aim_projection_ball, 0.1
        )

        # # Transicao para caso esteja longe da bola ( > 20 cm)
        far_to_ball_transition = OnNextTo(
            self.robot, aim_projection_ball, 0.3, True
        )

        angle_to_goal_transition = OnAttackerPushTrigger(
            self.robot, self.match
        )

        on_defensive_transition = OnDefensiveTransitionTrigger(
            self.robot, self.match, True, 0.25
        )

        out_defensive_transition = OnDefensiveTransitionTrigger(
            self.robot, self.match, False, 0.25
        )

        # Adiciona caminhos de ida e volta com transicoes
        astar.add_transition(next_to_ball_transition, limitcycle)

        limitcycle.add_transition(far_to_ball_transition, astar)
        limitcycle.add_transition(angle_to_goal_transition, push_potentialfield)

        push_potentialfield.add_transition(far_to_ball_transition, astar)

        astar.add_transition(on_defensive_transition, wait)
        limitcycle.add_transition(on_defensive_transition, wait)
        push_potentialfield.add_transition(on_defensive_transition, wait)

        wait.add_transition(out_defensive_transition, astar)

        # Estado inicial é o astar
        self.playerbook.set_play(astar)
        # self.playerbook.set_play(push_potentialfield)

    def decide(self):
        res = self.playerbook.update()
        return res
