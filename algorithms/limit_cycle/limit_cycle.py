import math
from collections import namedtuple

def discriminant(a, b, c, o):
    '''
    this function is used to evaluate if a line is tangent to a circle
    if dscr > 0 then the obstacle is blocking the way to the target
    '''
    k = 1 + (a**2/b**2)
    l = -2*o.x + (2*a*c)/b**2 + (2*a*o.y)/b
    m = o.x**2 + o.y**2 - o.radius**2 + c**2/b**2 + (2*c*o.y)/b
    dscr = l**2 -4*k*m
    return dscr

def dist(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def filter_func(a, b, c, r, t, o):
    '''
    checks if the obstacles are in the way of the robot to the target
    '''
    return discriminant(a, b, c, o) > 0 and dist(o, r) < dist(r, t) and dist(o, t) < dist(r, t)

Obstacle = namedtuple('Obstacle', ['x', 'y', 'radius', 'force_clockwise'])
Target = namedtuple('Target', ['x', 'y'])

class LimitCycle(object):
    def __init__(self, match):
        '''
        - obstacles:        list of the obstacles, not sorted yet
        - target_is_ball:   if the ball is the target, two virtual obstacles are added
                            to make the robot head to the goal when arriving
        '''
        self.match = match
        self.robot = None
        self.target = None
        self.obstacles = []

        if self.match.game.vision._fps:
            self._fps = self.match.game.vision._fps
        else:
            self._fps = 60

        self.dt = 1/self._fps

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def set_target(self, target):
        self.target = Target(*target)

    def add_obstacle(self, obstacle: Obstacle):
        self.obstacles.append(Obstacle(*obstacle))

    def contour(self, a, b, c, obstacle: Obstacle, fitness):
        '''
        this is the method used to avoid obstacles,
        based on the limit-cycle navigation from the soccer robotics book
        '''
        dx = self.robot.x - obstacle.x
        dy = self.robot.y - obstacle.y
        
        '''
        this multiplier is used to increase/decrease the fitness of the path around the obstacle
        based on the 'fitness' variable taken as a parameter
        '''
        mlt = int(fitness/math.sqrt(dx**2 + dy**2))

        '''
        if the obstacle is a virtual one the rotation direction shouldn't change
        '''
        if obstacle.force_clockwise == 0:
            d = (a*obstacle.x + b*obstacle.y + c)/math.sqrt(a**2 + b**2)
        else:
            d = obstacle.force_clockwise

        ddx =  (d/abs(d))*dy + mlt*dx*(obstacle.radius**2 - dx**2 - dy**2)
        ddy = -(d/abs(d))*dx + mlt*dy*(obstacle.radius**2 - dx**2 - dy**2)

        return (self.robot.x + self.dt*ddx, self.robot.y + self.dt*ddy)

    def compute(self, robot, fitness=15):
        self.robot = robot
        '''
        a, b and c are the indexes of a linear equation: a*x + b*y + c = 0
        '''
        a = self.target.y - self.robot.y
        b = self.robot.x - self.target.x
        c = self.target.x*self.robot.y - self.target.y*self.robot.x

        '''
        here all the non-disturbing obstacles are filtered from the obstacles list,
        using the filter function previously defined
        '''
        self.obstacles = list(filter(lambda o: filter_func(a, b, c, self.robot, self.target, o), self.obstacles))

        '''
        here the obstacles are sorted by their distance from the robot, ascending
        '''
        self.obstacles.sort(key=lambda o: math.sqrt((o.x - self.robot.x)**2 + (o.y - self.robot.y)**2))

        '''
        here we have the conditional, if there are obstacles to contour we do so, else we go to the target
        '''
        if len(self.obstacles) > 0:
            return self.contour(a, b, c, self.obstacles[0], fitness)

        else:
            dx = self.target.x - self.robot.x
            r_x = self.robot.x + self.dt*(dx/abs(dx))
            r_y = (-a*r_x - c)/b

            return r_x, r_y
