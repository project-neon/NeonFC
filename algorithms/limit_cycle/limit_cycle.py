import math
from controller import UniController

def discriminant(a, b, c, o):
    '''
    this function is used to evaluate if a line is tangent to a circle
    if dscr > 0 then the obstacle is blocking the way to the target
    '''
    k = 1 + (a**2/b**2)
    l = -2*o.x + (2*a*c)/b**2 + (2*a*o.y)/b
    m = o.x**2 + o.y**2 - o.r**2 + c**2/b**2 + (2*c*o.y)/b
    dscr = l**2 -4*k*m
    return dscr

def dist(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def filter_func(a, b, c, r, t, o):
    '''
    checks if the obstacles are in the way of the robot to the target
    '''
    return discriminant(a, b, c, o) > 0 and dist(o, r) < dist(r, t) and dist(o, t) < dist(r, t)

class LimitCycle(object):
    def __init__(self, strategy, target_is_ball=True):
        '''
        - obstacles:        list of the obstacles, not sorted yet
        - target_is_ball:   if the ball is the target, two virtual obstacles are added
                            to make the robot head to the goal when arriving
        '''
        self.strategy = strategy
        self.match = self.strategy.match
        self.target_is_ball = target_is_ball

        if self.match.game.vision._fps:
            self._fps = self.match.game.vision._fps
        else:
            self._fps = 60

        self.dt = 1/self._fps

        self.field_w, self.field_h = self.match.game.field.get_dimensions()

    def contour(self, a, b, c, obst, idx=15):
        '''
        this is the method used to avoid obstacles,
        based on the limit-cycle navigation from the soccer robotics book
        '''
        dx = self.robot.x - obst.x
        dy = self.robot.y - obst.y
        
        '''
        this multiplier is used to increase/decrease the fitness of the path around the obstacle
        based on the idx variable taken as a parameter
        '''
        mlt = int(idx/math.sqrt(dx**2 + dy**2))

        '''
        if the obstacle is a virtual one the rotation direction shouldn't change
        '''
        if obst.is_vo:
            if obst.side == "R": #cw
                d = 1
            else: # ccw
                d = -1
        else:
            d = (a*obst.x + b*obst.y + c)/math.sqrt(a**2 + b**2)

        ddx =  (d/abs(d))*dy + mlt*dx*(obst.r**2 - dx**2 - dy**2)
        ddy = -(d/abs(d))*dx + mlt*dy*(obst.r**2 - dx**2 - dy**2)

        '''
        Unicontroller
        '''
        if self.strategy.controller.__class__ is UniController:
            return math.atan2(ddy, ddx)

        '''
        PID_Control
        '''
        return (self.robot.x + self.dt*ddx, self.robot.y + self.dt*ddy)

    def update(self, robot, target, obstacles, target_is_ball=True):
        self.target_is_ball = target_is_ball
        self.robot = robot
        self.target = target
        self.obstacles = obstacles

    def compute(self):
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
        todo: remove the addition of virtual obstacles conditional from here,
              this is up to the strategy not part of the algorithm itself
        '''
        if self.target_is_ball:
            '''
            - m:    angle of the line perpendicular to the line between the ball and
                    the center of the goal
            - p:    distance of the virtual obstacles to the ball / radius of the virtual obstacle
            - vo:   virtual obstacle
            - j:    this is the angle between the ball and the center of the goal
            - m:    the normal angle perpendicular to j
            - r:    radius of the ball
            '''
            aim_point = [self.field_h/2, self.field_w]

            j = math.atan2(aim_point[0] - self.target.y, aim_point[1] - self.target.x)
            m = j + math.pi/2
            p = 0.1

            r =  .02 #(.0427)/2

            '''
            the terms r*cos(j) and r*sin(j) are subtracted to move
            the center of the obstacles behind the ball instead of its center
            '''
            if self.robot.y < math.tan(j)*(self.robot.x - self.target.x) + self.target.y:
                vo = Obstacle(self.target.x - p*math.cos(m) - r*math.cos(j), self.target.y - p*math.sin(m) - r*math.sin(j), p, side="R", is_vo=True)
            else:
                vo = Obstacle(self.target.x + p*math.cos(m) - r*math.cos(j), self.target.y + p*math.sin(m) - r*math.sin(j), p, side="L", is_vo=True)
            self.obstacles.append(vo)

        '''
        here the obstacles are sorted by their distance from the robot, ascending
        '''
        self.obstacles.sort(key=lambda o: math.sqrt((o.x - self.robot.x)**2 + (o.y - self.robot.y)**2))

        if len(self.obstacles) > 0:
            return self.contour(a, b, c, self.obstacles[0])

        else:

            dx = self.target.x - self.robot.x
            r_x = self.robot.x + self.dt*(dx/abs(dx))
            r_y = (-a*r_x - c)/b

            if self.strategy.controller.__class__ is UniController:
                dy = self.target.y - self.robot.y
                return math.atan2(dy, dx)

            return (r_x, r_y)

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Obstacle(Point):
    def __init__(self, x, y, r, side=None, is_vo=False):
        super().__init__(x, y)
        self.r = r
        self.side = side
        self.is_vo = is_vo
