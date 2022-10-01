from strategy.BaseStrategy import Strategy

class Attacker(Strategy):
	def __init__(self, match):
		super().__init__(match, "test2")
	def start(self, robot=None):
		super().start(robot=robot)
		
		self.seek = PotentialFild(self.match,name="SeekBehavior")
		self.aim = PotentialField(self.match, name="AimBehavior")
		
		self.seek.add_field(
		    PointField(
		        self.match,
		        target = lambda m : (m.ball.x, m.ball.y),
		        radius = .14,
		        decay = lambda x: x**2,
		        multiplier = .5
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
		            radius = .15,
		            radius_max = .15,
		            decay = lambda x: 1,
		            multiplier = -.5
		        )
		    )
		
	def decide(self):
        	return self.seek.compute([self.robot.x, self.robot.y])
