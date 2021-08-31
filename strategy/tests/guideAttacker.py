import math

from strategy.BaseStrategy import Strategy


from algorithms.potential_fields.fields import LineField, PotentialField, PointField

# class Attacker(DebugPotentialFieldStrategy):


class My_Attacker(Strategy):
    def __init__(self, match):
        super().__init__(match, "SimpleAttacker")
    
    def start(self, robot=None):
        super().start(robot=robot)

        self.behaviour = None

        self.seek = PotentialField(self.match,name="SeekBehaviour")

        self.aim = PotentialField(self.match,name="AimBehaviour")

        def on_angle20(m, f=self.match.game.field):
            field = f.get_dimensions()
            angle_ball_goal = math.atan2((field[1]/2 - m.ball.y), (field[0] - m.ball.x))
            ball_to_goal = (
                m.ball.x - math.cos(angle_ball_goal) * 0.2,
                m.ball.y - math.sin(angle_ball_goal) * 0.2
            )

            return ball_to_goal

        self.seek.add_field(
            PointField(
                self.match,
                target = on_angle20,
                radius = .05,
                decay = lambda x: x**4,
                multiplier = .75
            )
        )

        self.seek.add_field(
            PointField(
                self.match,
                target = lambda m : (m.ball.x, m.ball.y),
                radius = .12,
                radius_max = .12,
                decay = lambda x: 1,
                multiplier = -.75
            )
        )

        for robot in self.match.robots + self.match.opposites:
            if robot.get_name() == self.robot.get_name():
                continue        
            self.seek.add_field(
                    PointField(
                        self.match,
                        target = lambda m, r=robot: (
                            r.x,
                            r.y
                        ),
                        radius = .2,
                        radius_max = .2,
                        decay = lambda x: 1,
                        multiplier = -.75
                    )
                )

        def on_angle35(m, f=self.match.game.field):
            field = f.get_dimensions()
            angle_ball_goal = math.atan2((field[0] - m.ball.x), (field[1]/2 - m.ball.y))
            ball_to_goal = (
                m.ball.x - math.cos(angle_ball_goal) * 0.035,
                m.ball.y - math.sin(angle_ball_goal) * 0.035
            )

            return ball_to_goal

        self.aim.add_field(
            PointField(
                self.match,
                target = on_angle35,
                radius = .015,
                decay = lambda x: x,
                multiplier = .65
            )
        )

        self.aim.add_field(
            LineField(
                self.match,
                target = lambda m : (m.ball.x, m.ball.y),
                theta = lambda m, f=self.match.game.field: math.atan2((f.get_dimensions()[0] - m.ball.x), (f.get_dimensions()[1]/2 - m.ball.y)) + math.pi/2,
                line_size = .5,
                line_size_single_size=True,
                line_dist = .1,
                line_dist_max = .1,
                decay = lambda x: 1,
                multiplier = .1
            )
        )
            
            
    
    def decide(self):
        robot = self.robot
        ball = self.match.ball
        field = self.match.game.field.get_dimensions()

        angle_ball_goal = math.atan2((field[1]/2 - ball.y), (field[0] + 0.2 - ball.x))
        ball_to_goal = (
            ball.x - math.cos(angle_ball_goal) * 0.2,
            ball.y - math.sin(angle_ball_goal) * 0.2
        )
        is_near = lambda o, t, err: o - err <= t <= o + err

        if self.behaviour == None:
            self.behaviour = self.seek
            
        elif self.behaviour == self.seek:
            if (
                is_near(robot.x, ball_to_goal[0], 0.04) 
                and is_near(robot.y, ball_to_goal[1], 0.05)):
                self.behaviour = self.aim 
            else:
                pass
        elif self.behaviour == self.aim:
            if robot.x >= self.match.ball.x:
                self.behaviour = self.seek
        
        #return super().decide(self.behaviour)
        return self.behaviour.compute([self.robot.x, self.robot.y])