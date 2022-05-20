import math
import algorithms

from strategy.BaseStrategy import Strategy
from strategy.DebugTools import DebugPotentialFieldStrategy
from algorithms.potential_fields.plotter import PotentialDataExporter
import controller

# class Univector_Field(DebugPotentialFieldStrategy):
class Univector_Field(Strategy):
    def __init__(self, match, name="UVF-2",plot_field=False):
        super().__init__(match, name, controller=controller.TwoSidesLQR)

        self.plot_field = plot_field

    
    def start(self, robot=None):
        super().start(robot=robot)

        if self.plot_field:
            self.exporter = PotentialDataExporter(self.robot.get_name())
        else:
            self.exporter = False

        self.seek = algorithms.fields.PotentialField(
            self.match,
            name="{}|SeekBehaviour".format(self.__class__)
        )

        radius_e = 0.08 # radius_espiral

        self.field_w, self.field_h = self.match.game.field.get_dimensions()
    

        # if dist between ball and robot > radius
        def upper_point(m):
            goal_y = self.field_h/2
            goal_x = self.field_w

            point_x = m.ball.x + math.cos(math.atan2(goal_y-m.ball.y, goal_x-m.ball.x)+math.pi/2)*radius_e
            point_y = m.ball.y + math.sin(math.atan2(goal_y-m.ball.y, goal_x-m.ball.x)+math.pi/2)*radius_e

            return [point_x-0.01, point_y]

                
        def bottom_point(m):
            goal_y = self.field_h/2
            goal_x = self.field_w

            point_x = m.ball.x - math.cos(math.atan2(goal_y-m.ball.y, goal_x-m.ball.x)+math.pi/2)*radius_e
            point_y = m.ball.y - math.sin(math.atan2(goal_y-m.ball.y, goal_x-m.ball.x)+math.pi/2)*radius_e

            return [point_x-0.01, point_y]


        # to calculate potentential fiels multiplier
        yl = upper_point(self.match)[1] + radius_e
        yr = bottom_point(self.match)[1] - radius_e
        

        # multiplier calculus
        # if y is inside the spiral radius
        def move_to_goal_between_points():
            multiplier = ((yl * bottom_point(self.match)[1] + (yr * upper_point(self.match)[1]))/(2*radius_e))
            
            return multiplier

        # if y is under the spiral radius
        def move_to_goal_under_point():
            multiplier_ = bottom_point(self.match)[1]
            multiplier = max(1, multiplier_)

            return multiplier

        # if y is above the spiral radius
        def move_to_goal_above_point():
            multiplier_ =  upper_point(self.match)[1]
            multiplier = max(1, multiplier_)

            return multiplier


        self.seek.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=upper_point,                                                                                                                                                                                                                                                                                                                                          
                radius = radius_e,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = move_to_goal_above_point()
            )
        )

        self.seek.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target=bottom_point,                                                                                                                                                                                                                                                                                                                                          
                radius = radius_e,
                radius_max = 2,
                clockwise = False,
                decay=lambda x: 1,
                multiplier = move_to_goal_under_point()
            )
        )

        self.seek.add_field(
            algorithms.fields.PointField(
                self.match,
                target = lambda m: (m.ball.x + 0.08, m.ball.y),
                radius = 0.05,
                decay = lambda x: -1,
                multiplier = 0.9
            )
        )

        """self.seek.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target= upper_point,                                                                                                                                                                                                                                                                                                                                          
                radius = radius_e,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = move_to_goal_between_points()
            )
        )

        self.seek.add_field(
            algorithms.fields.TangentialField(
                self.match,
                target= bottom_point,                                                                                                                                                                                                                                                                                                                                          
                radius = radius_e,
                radius_max = 2,
                clockwise = True,
                decay=lambda x: 1,
                multiplier = move_to_goal_between_points()
            )
        )"""

    
    def decide(self, x=None, y=None):
        if x:
            self.robot.x = x
        if y:
            self.robot.y = y
        
        behaviour = self.seek

        if self.exporter:
            self.exporter.export(behaviour, self.robot, self.match.ball)
            return (0, 0)
        
        return behaviour.compute([self.robot.x, self.robot.y])