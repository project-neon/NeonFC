import os, math, time, random, copy

ROBOT_WIDTH = 0.08
ROBOT_RADIUS = 0.0375
SAFE_DIST = 0.0375
BARRIER_RADIUS = 0.0375
MAX_VELOCITY = 0.5
MAX_ACCELERATION = 0.5
STEPS_AHEAD_TO_PLAN = 10

class DynamicWindowApproach:

    def __init__(self, robot, game):
        self.robot = robot
        self.game = game
        self.vL = 0.00
        self.vR = 0.00
    
    def get_obstacles(self):
        return self.game.match.opposites

    def predict_position(self, vL, vR, x, y, theta, deltat):
        # Simple special cases
        # Straight line motion
        if (round (vL,3) == round(vR,3)):
            xnew = x + vL * deltat * math.cos(theta)
            ynew = y + vL * deltat * math.sin(theta)
            thetanew = theta
        # Pure rotation motion
        elif (round(vL,3) == -round(vR,3)):
            xnew = x
            ynew = y
            thetanew = theta + ((vR - vL) * deltat / ROBOT_WIDTH)
        else:
            # Rotation and arc angle of general circular motion
            # Using equations given in Lecture 2
            R = ROBOT_WIDTH / 2.0 * (vR + vL) / (vR - vL)
            deltatheta = (vR - vL) * deltat / ROBOT_WIDTH
            xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
            ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
            thetanew = theta + deltatheta

        return (xnew, ynew, thetanew)

    def calculate_closest_obstacles_distance(self, x, y):
        closestdist = 100000.0  
        # Calculate distance to closest obstacle
        opposites = self.get_obstacles()
        for (i,opposites) in enumerate(opposites):
            dx = opposites.x - x
            dy = opposites.y - y
            d = math.sqrt(dx**2 + dy**2)
            # Distance between closest touching point of circular robot and circular barrier
            dist = d - BARRIER_RADIUS - ROBOT_RADIUS
            if (dist < closestdist):
                closestdist = dist
        return closestdist

    def get_target(self):
        return self.game.match.ball

    def set_desired(self, desired):
        pass

    def update(self):
        return self.get_best_path()

    def decide(self):
        return [0, 0]

    def start(self, robot):
        pass

    def get_best_path(self):

        opposites = self.get_obstacles()
        if (self.game.vision._fps > 0):
            dt = 1.0 / self.game.vision._fps
        else:
            dt = 1.0/60

        BEST_BENEFIT = -100000
        FORWARD_WEIGHT = 12
        OBSTACLE_WEIGHT = 6666
        TAU = dt * STEPS_AHEAD_TO_PLAN

        vLpossiblearray = (self.vL - MAX_ACCELERATION * dt, self.vL, self.vL + MAX_ACCELERATION * dt)
        vRpossiblearray = (self.vR - MAX_ACCELERATION * dt, self.vR, self.vR + MAX_ACCELERATION * dt)

        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                # We can only choose an action if it's within velocity limits
                if (vLpossible <= MAX_VELOCITY and vRpossible <= MAX_VELOCITY and vLpossible >= -MAX_VELOCITY and vRpossible >= -MAX_VELOCITY):
                    # Predict new position in TAU seconds
                    (xpredict, ypredict, thetapredict) = self.predict_position(vLpossible, vRpossible, self.robot.x, self.robot.y, self.robot.theta, TAU)
                    # What is the distance to the closest obstacle from this possible position?
                    distanceToObstacle = self.calculate_closest_obstacles_distance(xpredict, ypredict)
                    # Calculate how much close we've moved to target location
                    previousTargetDistance = math.sqrt((self.robot.x - self.get_target().x)** 2 + (self.robot.y - self.get_target().y) ** 2)
                    newTargetDistance = math.sqrt((xpredict - self.get_target().x) ** 2 + (ypredict - self.get_target().y) ** 2)
                    distanceForward = previousTargetDistance - newTargetDistance
                    # Alternative: how far have I moved forwards?
                    # distanceForward = xpredict - x
                    # Positive benefit
                    distanceBenefit = FORWARD_WEIGHT * distanceForward
                    # Negative cost: once we are less than SAFE_DIST from collision, linearly increasing cost
                    if (distanceToObstacle < SAFE_DIST):
                        obstacleCost = OBSTACLE_WEIGHT * (SAFE_DIST - distanceToObstacle)
                    else:
                        obstacleCost = 0.0
                    # Total benefit function to optimise
                    benefit = distanceBenefit - obstacleCost
                    if (benefit > BEST_BENEFIT):
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        BEST_BENEFIT = benefit
                    
        self.vL = vLchosen
        self.vR = vRchosen

        return self.vL, self.vR
        
