
# Planning
# Dynamic Window Approach (Local Planning) with moving obstacles
# Andrew Davison 2019
import os, math, time, random, copy


# Constants and variables
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.1
ROBOTRADIUS = 0.10
W = 2 * ROBOTRADIUS # width of robot
SAFEDIST = ROBOTRADIUS      # used in the cost function for avoiding obstacles

MAXVELOCITY = 0.5     #ms^(-1) max speed of each wheel
MAXACCELERATION = 0.5 #ms^(-2) max rate we can change speed of each wheel


BARRIERVELOCITYRANGE = 0.15


# The region we will fill with obstacles
PLAYFIELDCORNERS = (-4.0, -3.0, 4.0, 3.0)



# Starting pose of robot
x = PLAYFIELDCORNERS[0] - 0.5
y = 0.0
theta = 0.0

# Use for displaying a trail of the robot's positions
locationhistory = []

# Starting wheel velocities
vL = 0.00
vR = 0.00

# Timestep delta to run control and simulation at
dt = 0.1
STEPSAHEADTOPLAN = 10
TAU = dt * STEPSAHEADTOPLAN

# Barrier (obstacle) locations
barriers = []
# barrier contents are (bx, by, visibilitymask)
# Generate some initial random barriers

                

        
# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y

# set the width and height of the screen (pixels)
WIDTH = 1500
HEIGHT = 1000

size = [WIDTH, HEIGHT]
black = (20,20,40)
lightblue = (0,120,255)
darkblue = (0,40,160)
red = (255,100,0)
white = (255,255,255)
blue = (0,0,255)
grey = (70,70,70)
k = 160 # pixels per metre for graphics

# Screen centre will correspond to (x, y) = (0, 0)
u0 = WIDTH / 2
v0 = HEIGHT / 2




# Initialise Pygame display screen
#screen = pygame.display.set_mode(size)
# This makes the normal mouse pointer invisible in graphics window
#pygame.mouse.set_visible(0)


# Array for path choices use for graphics 
pathstodraw = []


# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
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
                thetanew = theta + ((vR - vL) * deltat / W)
        else:
                # Rotation and arc angle of general circular motion
                # Using equations given in Lecture 2
                R = W / 2.0 * (vR + vL) / (vR - vL)
                deltatheta = (vR - vL) * deltat / W
                xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
                ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
                thetanew = theta + deltatheta

                # To calculate parameters for arc drawing (complicated Pygame stuff, don't worry)
                # We need centre of circle
                (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
                # Turn this into Rect
                Rabs = abs(R)
                ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
                if (R > 0):
                        start_angle = theta - math.pi/2.0
                else:
                        start_angle = theta + math.pi/2.0
                stop_angle = start_angle + deltatheta

        return (xnew, ynew, thetanew)

# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
        closestdist = 100000.0  
        # Calculate distance to closest obstacle
        for (i,barrier) in enumerate(barriers):
                if (i != targetindex):
                        dx = barrier[0] - x
                        dy = barrier[1] - y
                        d = math.sqrt(dx**2 + dy**2)
                        # Distance between closest touching point of circular robot and circular barrier
                        dist = d - BARRIERRADIUS -      ROBOTRADIUS
                        if (dist < closestdist):
                                closestdist = dist
        return closestdist

goflag = 0

# Main loop
while(1):
        
        # Planning
        # We want to find the best benefit where we have a positive component for closeness to target,
        # and a negative component for closeness to obstacles, for each of a choice of possible actions
        bestBenefit = -100000
        FORWARDWEIGHT = 12
        OBSTACLEWEIGHT = 6666

        
        # Copy of barriers so we can predict their positions
        barrierscopy = copy.deepcopy(barriers)

        for i in range(STEPSAHEADTOPLAN):
                moveBarriers(dt)

        
        # Range of possible motions: each of vL and vR could go up or down a bit
        vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
        vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
        pathstodraw = [] # We will store path details here for plotting later
        newpositionstodraw = [] # Also for possible plotting of robot end positions
        for vLpossible in vLpossiblearray:
                for vRpossible in vRpossiblearray:
                        # We can only choose an action if it's within velocity limits
                        if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                                # Predict new position in TAU seconds
                                (xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
                                pathstodraw.append(path)
                                newpositionstodraw.append((xpredict, ypredict))
                                # What is the distance to the closest obstacle from this possible position?
                                distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
                                # Calculate how much close we've moved to target location
                                previousTargetDistance = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
                                newTargetDistance = math.sqrt((xpredict - barriers[targetindex][0])**2 + (ypredict - barriers[targetindex][1])**2)
                                distanceForward = previousTargetDistance - newTargetDistance
                                # Alternative: how far have I moved forwards?
                                # distanceForward = xpredict - x
                                # Positive benefit
                                distanceBenefit = FORWARDWEIGHT * distanceForward
                                # Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
                                if (distanceToObstacle < SAFEDIST):
                                        obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                                else:
                                        obstacleCost = 0.0
                                # Total benefit function to optimise
                                benefit = distanceBenefit - obstacleCost
                                if (benefit > bestBenefit):
                                        vLchosen = vLpossible
                                        vRchosen = vRpossible
                                        bestBenefit = benefit
        vL = vLchosen
        vR = vRchosen
        
        barriers = copy.deepcopy(barrierscopy)


        # Actually now move robot based on chosen vL and vR
        (x, y, theta, tmppath) = predictPosition(vL, vR, x, y, theta, dt)


        moveBarriers(dt)

        
        # Wraparound: check if robot has reached target; if so reset it to the other side, randomise
        # target position and add some more barriers to go again
        disttotarget = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
        if (disttotarget < (BARRIERRADIUS + ROBOTRADIUS)):
                # Add new barriers
                for i in range(3):
                        (bx, by) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]))
                        (bx, by, vx, vy) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]), random.uniform(-BARRIERVELOCITYRANGE, BARRIERVELOCITYRANGE), random.uniform(-BARRIERVELOCITYRANGE, BARRIERVELOCITYRANGE))
                        barrier = [bx, by, vx, vy]
                        barriers.append(barrier)
                targetindex = random.randint(0,len(barriers)-1)

                
                # Reset trail
                locationhistory = []

                
        # Sleeping dt here runs simulation in real-time
        time.sleep(dt / 50)
        #time.sleep(1.0)
        
