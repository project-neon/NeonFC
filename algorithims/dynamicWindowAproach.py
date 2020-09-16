def predictPosition(vL, vR, x, y, theta, deltat):
        if (round (vL,3) == round(vR,3)):
                xnew = x + vL * deltat * math.cos(theta)
                ynew = y + vL * deltat * math.sin(theta)
                thetanew = theta
        elif (round(vL,3) == -round(vR,3)):
                xnew = x
                ynew = y
                thetanew = theta + ((vR - vL) * deltat / W)
        else:
                R = W / 2.0 * (vR + vL) / (vR - vL)
                deltatheta = (vR - vL) * deltat / W
                xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
                ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
                thetanew = theta + deltatheta
                (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
                Rabs = abs(R)
                ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
                if (R > 0):
                        start_angle = theta - math.pi/2.0
                else:
                        start_angle = theta + math.pi/2.0
                stop_angle = start_angle + deltatheta

        return (xnew, ynew, thetanew)

def calculateClosestObstacleDistance(x, y):
        closestdist = 100000.0  
        for (i,barrier) in enumerate(barriers):
                if (i != targetindex):
                        dx = barrier[0] - x
                        dy = barrier[1] - y
                        d = math.sqrt(dx**2 + dy**2)
                        dist = d - BARRIERRADIUS - ROBOTRADIUS
                        if (dist < closestdist):
                                closestdist = dist
        return closestdist

goflag = 0

while(True):
        bestBenefit = -100000
        FORWARDWEIGHT = 12
        OBSTACLEWEIGHT = 6666

        barrierscopy = copy.deepcopy(barriers)

        for i in range(STEPSAHEADTOPLAN):
                moveBarriers(dt)

        vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
        vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
        pathstodraw = []
        newpositionstodraw = []
        for vLpossible in vLpossiblearray:

                        if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                                (xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
                                pathstodraw.append(path)
                                newpositionstodraw.append((xpredict, ypredict))
                                distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
                                previousTargetDistance = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
                                newTargetDistance = math.sqrt((xpredict - barriers[targetindex][0])**2 + (ypredict - barriers[targetindex][1])**2)
                                distanceForward = previousTargetDistance - newTargetDistance
                                distanceBenefit = FORWARDWEIGHT * distanceForward
                                if (distanceToObstacle < SAFEDIST):
                                        obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                                else:
                                        obstacleCost = 0.0
                                benefit = distanceBenefit - obstacleCost
                                if (benefit > bestBenefit):
                                        vLchosen = vLpossible
                                        vRchosen = vRpossible
                                        bestBenefit = benefit
        vL = vLchosen
        vR = vRchosen
        
        barriers = copy.deepcopy(barrierscopy)

        (x, y, theta, tmppath) = predictPosition(vL, vR, x, y, theta, dt)


        moveBarriers(dt)
        disttotarget = math.sqrt((x - barriers[targetindex][0])**2 + (y - barriers[targetindex][1])**2)
        if (disttotarget < (BARRIERRADIUS + ROBOTRADIUS)):

        time.sleep(dt / 50)