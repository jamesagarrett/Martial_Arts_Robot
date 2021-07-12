##  
##  James Garrett
##
##  Martial_Arts_Robot 
##  Last Updated: July 12, 2021
##
##  maneuverAnalysis.py
##  Last Updated: July 12, 2021
##
##  Determine the best course of action for maneuvering the machine back within
##  the desired ranges described in globals.py.
##

from math import atan, ceil, cos, degrees, floor, radians, sin, sqrt, tan 
from bisect import bisect_left                             
from helperFunctions import getCollinearDistance

##Module for performing reposition of the machine, based on sensor data 
##collection and analysis.
from repositionMachine import moveFromObject, moveToOpponent

##THESE VALUES ARE NOT TO BE CHANGED!
from globals import ANGLE_ERR,\
                    MACH_RADIUS,\
                    OPT_DISTANCE,\
                    PATH_ZONE,\
                    SNS_MAX_DISTANCE,\
                    SNS_MIN_DISTANCE,\
                    SNS_OPT_DISTANCE,\
                    SUF_DISTANCE

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:      calculateObjMovement
## called by: sensorAnalysis.interpretData()
##            maneuverAnalysis.calculateObjMovement()
## passed:    int[] activeAngles, float[] activeDistances,
##            int opponentAngle, float[] distanceValues
## returns:   nothing
## calls:     getCollinearDistance()
##            maneuverAnalysis.calculateObjMovement(),
##                             findMoveAngle(), 
##                             findMoveDistance(), 
##                             findNewOppDist(),
##                             getPathAngles(),
##                             isPathClear()
##            repositionMachine.moveFromObject()            
##
## Determine how to reposition when too close to an object*
## including the opponent.                                *
## ********************************************************
def calculateObjMovement(objAngles, objDistances, oppAngle, allDistances):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################
    
    maneuverAngle = 0           ##The angle the machine will move at when
                                ##repositioning.

    maneuverDistance = 0.0      ##The distance the machine will travel when 
                                ##repositioning.

    maneuverFound = False       ##Set to True once maneuverAngle and
                                ##maneuverDistance have been determined.

    moveObjDistance = 0.0       ##The distance to the nearest object in the
                                ##direction of maneuverAngle.

    newOppDistance = 0.0        ##The distance from the machine to the opponent
                                ##after traveling the maneuverDistance in the 
                                ##direction of maneuverAngle.

    leftClosestAngle = \
    rightClosestAngle = 0       ##Used to find the closest angles to 
                                ##maneuverAngle if its corresponding distance
                                ##value is less than MACH_RADIUS; the average is
                                ##then used, so long as they are not too far 
                                ##from the original angle.

    pathAngles = []             ##Any angle in the path of the machine when  
                                ##repositioning; that is, an angle at which an 
                                ##object at said angle, if not far enough, could
                                ##be contacted by the machine while maneuvering
                                ##or could be considered too close to the
                                ##machine after maneuvering.

    clearPath = False           ##Set to True if isPathClear() returns True, 
                                ##finding no objects within the desired movement
                                ##path of the machine. 

    oppInRange = False          ##Set to True if the machine can move at
                                ##maneuverAngle using maneuverDistance without 
                                ##then being too close or far from the opponent.

    insertPoint = 0             ##The position in which an item is added to the
                                ##objAngles and objDistances lists.
    
    wallAnglesCount = 0         ##The amount of adjacent angles on either side
                                ##of the current value being added to 
                                ##objAngles. These angles will also be added
                                ##to objAngles if they are detecting the same
                                ##object as the original angle.

    adjustedDistance = 0        ##The returned value of getCollinearDistance(). 

    #####################################
    
    maneuverAngle = findMoveAngle(objAngles)

    if(maneuverAngle == -1):
        #print("Maneuver: CAN'T MOVE\n")
        #print("Active Angles:\n\n", objAngles, "\n\nActive Distances:\n\n", objDistances, "\n\n")
        return

    ##Retrieve the distance value at the associated maneuverAngle. If this value
    ##is detecting an object within the radius of the machine - such as a
    ##mechanical support beam - use the adjacent angles to estimate the 
    ##distance. If still not calculable, add maneuverAngle and its distance 
    ##to objAngles and objDistances respectively, and recursively call 
    ##calculateObjMovement() until maneuverAngle and manueverDistance can be
    ##determined.
    if(allDistances[maneuverAngle] > MACH_RADIUS):
        moveObjDistance = allDistances[maneuverAngle]
    else:
        if(maneuverAngle != 0):
            leftClosestAngle = maneuverAngle - 1
        else:
            leftClosestAngle = 359

        if(maneuverAngle != 359):
            rightClosestAngle = maneuverAngle + 1
        else:
            rightClosestAngle = 0

        for x in range(1, ANGLE_ERR):
            if(allDistances[leftClosestAngle] <= MACH_RADIUS):
                if(leftClosestAngle != 0):
                    leftClosestAngle -= 1
                else:
                    leftClosestAngle = 359
            if(allDistances[rightClosestAngle] <= MACH_RADIUS):
                if(rightClosestAngle != 359):
                    rightClosestAngle += 1
                else:
                    rightClosestAngle = 0

            if(allDistances[leftClosestAngle] > MACH_RADIUS 
                    < allDistances[rightClosestAngle]):
                moveObjDistance = (allDistances[leftClosestAngle] 
                                + allDistances[rightClosestAngle]) / 2
                break

    if(moveObjDistance == 0.0):
        insertPoint = bisect_left(objAngles, maneuverAngle)
        objAngles.insert(insertPoint, maneuverAngle)
        objDistances.insert(insertPoint, allDistances[maneuverAngle])
        calculateObjMovement(objAngles, objDistances, oppAngle, allDistances)
        return

    ##Once the best angle to traverse at is determined, find all additional 
    ##angles in the path of the machine when traveling in the direction of 
    ##maneuverAngle.

    pathAngles = getPathAngles(maneuverAngle)

    ##Check OPT_DISTANCE maneuverability.

    maneuverDistance = findMoveDistance(objAngles, objDistances, 
                                        maneuverAngle, OPT_DISTANCE)
        
    newOppDistance = findNewOppDist(maneuverAngle, maneuverDistance,
                                    oppAngle, allDistances[oppAngle])

    clearPath = isPathClear(maneuverAngle, pathAngles, allDistances,
                              MACH_RADIUS, maneuverDistance + MACH_RADIUS)

    oppInRange = SNS_MIN_DISTANCE <= newOppDistance <= SNS_MAX_DISTANCE

    if(clearPath and oppInRange):
        maneuverFound = True
    else:
        ##Check SUF_DISTANCE maneuverability.

        maneuverDistance = findMoveDistance(objAngles, objDistances, 
                                                maneuverAngle, SUF_DISTANCE) 

        newOppDistance = findNewOppDist(maneuverAngle, maneuverDistance,
                                        oppAngle, allDistances[oppAngle])
            
        clearPath = isPathClear(maneuverAngle, pathAngles, allDistances,
                                  MACH_RADIUS, maneuverDistance + MACH_RADIUS)

        oppInRange = SNS_MIN_DISTANCE <= newOppDistance <= SNS_MAX_DISTANCE

        if(clearPath and oppInRange):
            maneuverFound = True
        
        else:
            insertPoint = bisect_left(objAngles, maneuverAngle)
            objAngles.insert(insertPoint, maneuverAngle)
            objDistances.insert(insertPoint, moveObjDistance)

            wallAnglesCount = ceil(degrees(atan(MACH_RADIUS/moveObjDistance)))

            for x in range (maneuverAngle - wallAnglesCount, maneuverAngle
                            + wallAnglesCount + 1):
                
                if(x < 0):
                    x += 360
                elif(x > 359):
                    x -= 360
                
                adjustedDistance = getCollinearDistance(x, maneuverAngle,
                                                        moveObjDistance)

                if(MACH_RADIUS < allDistances[x] <= adjustedDistance):

                    insertPoint = bisect_left(objAngles, x)

                    if(len(objAngles) == 0 or insertPoint == 
                    len(objAngles) or x != objAngles[insertPoint]):
                        objAngles.insert(insertPoint, x)
                        objDistances.insert(insertPoint, allDistances[x])

    if(maneuverFound):
        #print("CAN MOVE\nManeuver: Angle -", maneuverAngle, "\tDistance -", maneuverDistance, "\n")
        #print("Active Angles:\n\n", objAngles, "\n\nActive Distances:\n\n", objDistances, "\n\n")
        moveFromObject(maneuverAngle, maneuverDistance, moveObjDistance,
                       pathAngles)
    else:
        calculateObjMovement(objAngles, objDistances, oppAngle, allDistances)

    return 

## ********************************************************
## name:      calculateOppMovement
## called by: sensorAnalysis.interpretData()
## passed:    int[] activeAngles, float[] activeDistances
## returns:   nothing
## calls:     maneuverAnalysis.getPathAngles(),
##                             isPathClear()
##            repositionMachine.moveToOpponent()
##
## Determine how to reposition when too far from the      *
## opponent.                                              *
## ********************************************************
def calculateOppMovement(oppAngles, oppDistances, allDistances):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################
    
    clearPath = False       ##Set to True if isPathClear() finds no objects
                            ##within the desired movement path of the machine.

    maneuverAngle = 0       ##The approximate midpoint of the opponent's 
                            ##location to move at when repositioning.

    maneuverDistance = 0.0  ##The distance the machine will travel when 
                            ##repositioning.
    
    pathAngles = []         ##Any angle in the path of the machine when it is
                            ##repositioning; that is, an angle at which an 
                            ##object at said angle, if not far enough, could be 
                            ##contacted by the machine while maneuvering.

    #####################################
    
    ##Due to the amount of lateral movement the opponent has at and beyond the 
    ##MAX_DISTANCE, there is the potential that moving directly toward the 
    ##DES_OPP_ANGLE could result in the machine not directly facing the opponent
    ##after moving. This is accounted for by finding the center-most point of
    ##the opponent, and using that angle as the maneuverAngle.
    for x in range (0, len(oppAngles)):
        maneuverAngle += oppAngles[x]

    maneuverAngle = floor(maneuverAngle/len(oppAngles))
    pathAngles = getPathAngles(maneuverAngle)

    maneuverDistance = allDistances[maneuverAngle] - SNS_OPT_DISTANCE

    clearPath = isPathClear(maneuverAngle, pathAngles, allDistances,
                            MACH_RADIUS, maneuverDistance + MACH_RADIUS)

    if(clearPath):
        #print("CAN MOVE\nManeuver: Angle -", maneuverAngle, "\tDistance -", maneuverDistance, "\n")
        #print("Active Angles:\n\n", oppAngles, "\n\nActive Distances:\n\n", oppDistances, "\n\n")
        moveToOpponent(maneuverAngle, pathAngles, oppAngles)
    #else:
        #print("Maneuver: CAN'T MOVE\n")
        #print("Active Angles:\n\n", oppAngles, "\n\nActive Distances:\n\n", oppDistances, "\n\n")
            
    return 

## ********************************************************
## name:      findMoveAngle
## called by: maneuverAnalysis.calculateObjMovement()
## passed:    int[] objAngles
## returns:   int maneuverAngle
## calls:     nobody 
##
## Determine the preferred directional angle for the      *
## machine to move when repositioning itself away from an *
## object or opponent considered too close.               *
## ********************************************************
def findMoveAngle(objAngles):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    objectsSpace = 0.0      ##The angle between any two consecutive objAngles 
                            ##values.

    largestSpace = 0.0      ##The largest angle between any two consecutive 
                            ##objAngles values.
    
    maneuverAngle = -1      ##The center-most angle within largestSpace, in 
                            ##which the machine will move toward when   
                            ##repositioning.

    #####################################

    ##Find the largest value - and thus the largest physical space to maneuver 
    ##within - between the list of objAngles values. The midpoint of this 
    ##largestSpace is where the machine will maneuver itself.
 
    for x in range (-len(objAngles), 0):      
        if(len(objAngles) == 1):    
            objectsSpace = 360
        elif(x == -1):              
            objectsSpace = 360 - (objAngles[-1] - objAngles[0])
        else:
            objectsSpace = objAngles[x+1] - objAngles[x]

        if(objectsSpace > largestSpace):
            largestSpace = objectsSpace 
            if(x == -1):
                if((objAngles[-1] + objAngles[0])/2 >= 180):
                    maneuverAngle = floor((objAngles[-1] \
                                    + objAngles[0])/2 - 180)
                else:
                    maneuverAngle = floor((objAngles[-1] \
                                    + objAngles[0])/2 + 180)
            else:
                maneuverAngle = floor((objAngles[x+1] + objAngles[x])/2)

    if(largestSpace < PATH_ZONE):
        maneuverAngle = -1

    return maneuverAngle

## ********************************************************
## name:      findMoveDistance
## called by: maneuverAnalysis.calculateObjMovement()
## passed:    int[] objAngles, float[] objDistances,
##            int maneuverAngle, float OPT_DISTANCE/SUF_DISTANCE
## returns:   float maneuverDistance
## calls:     nobody 
##
## Determine the required distance for the machine to     *
## travel while repositioning itself away from an object  *                  
## or opponent considered too close.                      *
## ********************************************************
def findMoveDistance(objAngles, objDistances, maneuverAngle, desiredDistance):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################
    
    maneuverDistance = 0.0      ##The distance the machine will travel when 
                                ##repositioning.

    objXComp = 0.0              ##The x(horizontal) component of an object's 
                                ##location relative to the machine.

    objYComp = 0.0              ##The y(vertical) component of an object's 
                                ##location relative to the machine.

    maneuverXComp = 0.0         ##The x(horizontal) component of the 
                                ##maneuverDistance value.

    maneuverYComp = 0.0         ##The y(vertical) component of the 
                                ##maneuverDistance value.

    a = b = c = 0.0             ##The 3 coefficients needed for the quadratic 
                                ##formula; used to calculate maneuverXComp.

    #####################################
    
    ##In order to calculate the value of maneuverDistance, we must know the 
    ##smallest distance between the machine and any given object that is too 
    ##close. The amount of movement required to reach the desiredDistance away
    ##from this object is our maneuverDistance.
        
    for x in range (0, len(objAngles)):
        
        ##Ignore values that were added to objAngles in calculateObjMovement()
        ##due to the fact that they - or their corresponding angles - were not 
        ##adequate for proper reposition. These distances were not originally  
        ##found as being too close to the machine, and thus do not need to be  
        ##accounted for.

        if(objDistances[x] - MACH_RADIUS >= desiredDistance 
                or objDistances[x] <= MACH_RADIUS):
            continue

        objXComp = cos(radians(objAngles[x])) * (objDistances[x] - MACH_RADIUS)
        objYComp = sin(radians(objAngles[x])) * (objDistances[x] - MACH_RADIUS)

        if(maneuverAngle == 90 or maneuverAngle == 270):
            maneuverXComp = 0.0
            maneuverYComp = sqrt(desiredDistance**2 - objXComp**2) \
                            - abs(objYComp)
            if(maneuverYComp > maneuverDistance):
                maneuverDistance = maneuverYComp   
        else:
            a = tan(radians(maneuverAngle))**2 + 1.0
            b = -2*objXComp - 2*objYComp*tan(radians(maneuverAngle))
            c = objXComp**2 + objYComp**2 - desiredDistance**2

            maneuverXComp = (-b + sqrt(b**2 - 4*a*c)) / (2*a)           

            ##Check if maneuverXComp is in the correct quadrant for the value 
            ##of maneuverAngle.
            if((maneuverXComp < 0 and 90 < maneuverAngle < 270) 
                    or (maneuverXComp > 0 and (0 <= maneuverAngle < 90 
                                               or 270 < maneuverAngle < 360))):
                maneuverYComp = maneuverXComp * tan(radians(maneuverAngle))
                if(sqrt(maneuverXComp**2 + maneuverYComp**2) 
                        > maneuverDistance):
                    maneuverDistance = sqrt(maneuverXComp**2 + maneuverYComp**2)

            #Recalculate maneuverXComp if in the incorrect quadrant.
            elif((maneuverXComp > 0 and 90 < maneuverAngle < 270) 
                    or (maneuverXComp < 0 and (0 <= maneuverAngle < 90 
                                               or 270 < maneuverAngle < 360))):
                maneuverXComp = (-b - sqrt(b**2 - 4*a*c)) / (2*a)
                maneuverYComp = maneuverXComp * tan(radians(maneuverAngle))
                if(sqrt(maneuverXComp**2 + maneuverYComp**2) 
                        > maneuverDistance):
                    maneuverDistance = sqrt(maneuverXComp**2 + maneuverYComp**2)
        
    return maneuverDistance

## ********************************************************
## name:      findNewOppDist
## called by: maneuverAnalysis.calculateObjMovement()
## passed:    int maneuverAngle, float maneuverDistance,
##            int oppAngle, float allDistances[oppAngle]
## returns:   float vectorDistance
## calls:     nobody 
##
## Determine the distance from the opponent to the machine*
## if reposition were to occur at the specified angle and *
## distance.                                              *
## ********************************************************
def findNewOppDist(maneuverAngle, maneuverDistance, oppAngle, oppDistance):
    
    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    oppXComp = 0.0              ##The x(horizontal) component of the oppDistance
                                ##value.

    oppYComp = 0.0              ##The y(vertical) component of the oppDistance
                                ##value.

    maneuverXComp = 0.0         ##The x(horizontal) component of the 
                                ##maneuverDistance value.

    maneuverYComp = 0.0         ##The y(vertical) component of the 
                                ##maneuverDistance value.

    vectorDistance = 0.0        ##The distance between the opponent and the 
                                ##machine after reposition.

    #####################################

    oppXComp = cos(radians(oppAngle)) * (oppDistance)
    oppYComp = sin(radians(oppAngle)) * (oppDistance)

    maneuverXComp = cos(radians(maneuverAngle)) * (maneuverDistance)
    maneuverYComp = sin(radians(maneuverAngle)) * (maneuverDistance)

    vectorDistance = sqrt((oppXComp - maneuverXComp)**2 
                          + (oppYComp - maneuverYComp)**2)

    return vectorDistance

## ********************************************************
## name:      getPathAngles
## called by: maneuverAnalysis.calculateObjMovement(),
##                             calculateOppMovement()
## passed:    int maneuverAngle
## returns:   int[] pathAngles
## calls:     nobody 
##
## Determine all angles within the path of the machine    *
## if repositioning were to occur at the specified angle. * 
## ********************************************************
def getPathAngles(maneuverAngle):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    pathAngles = [maneuverAngle]    ##List of angles to be returned that include
                                    ##all angles considered in the maneuvering 
                                    ##path of the machine as it looks to 
                                    ##reposition.
    
    #####################################

    for x in range (1, PATH_ZONE + 1):
        if(maneuverAngle-x >= 0):
            pathAngles.insert(0, maneuverAngle-x)
        else:
            pathAngles.insert(0, 360+(maneuverAngle-x))

        if(maneuverAngle+x <= 359):
            pathAngles.append(maneuverAngle+x)
        else:
            pathAngles.append((maneuverAngle+x)-360)

    return pathAngles

## ********************************************************
## name:      isPathClear
## called by: maneuverAnalysis.calculateObjMovement(),
##                             calculateOppMovement()
## passed:    int maneuverAngle, int[] pathAngles, 
##            float[] allDistances, float MACH_RADIUS,
##            float maneuverDistance + MACH_RADIUS
## returns:   bool
## calls:     nobody
##
## Determine if it is safe to reposition in the direction *
## of maneuverAngle, that is, no objects are in the       *
## perspective path of the machine or in a position in    *
## which said objects would be too close after            *
## repositioning.                                         *
## ********************************************************
def isPathClear(maneuverAngle, pathAngles, allDistances, xBound, yBound):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    coordAngle = 0.0        ##The angle between maneuverAngle and any given 
                            ##pathAngles value.

    xCoord = yCoord = 0.0   ##The x and y coordinate values of any pathAngles
                            ##value with respect to the value of coordAngle.
 
    #####################################

    for x in range (0, len(pathAngles)):

        if(allDistances[pathAngles[x]] <= MACH_RADIUS):
            continue
        
        if(abs(maneuverAngle - pathAngles[x]) > len(pathAngles)//2):
            if(maneuverAngle < pathAngles[x]):
                coordAngle = abs(360 + maneuverAngle - pathAngles[x])
            else:
                coordAngle = abs(360 + pathAngles[x] - maneuverAngle)
        else:
            coordAngle = abs(maneuverAngle - pathAngles[x])
        
        xCoord = sin(radians(coordAngle))*allDistances[pathAngles[x]]
        yCoord = cos(radians(coordAngle))*allDistances[pathAngles[x]]
        
        if(xCoord <= xBound and yCoord < yBound):
            return False

    return True


