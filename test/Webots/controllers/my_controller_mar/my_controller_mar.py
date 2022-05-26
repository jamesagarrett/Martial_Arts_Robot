# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import atan, ceil, degrees, floor
from bisect import bisect_left	

##Module that includes functions that are used throughout the project.
from helperFunctions import getCollinearDistance, getCartesianAngle

##Module to determine how the machine will reposition itself toward or
##away from the opponent.
from maneuverAnalysis import calculateObjMovement

##Module for performing repositioning of the machine, based on sensor data 
##collection and analysis.
from repositionMachine import moveToOpponent, rotateMachine

##THESE VALUES ARE NOT TO BE CHANGED!
from globals import BEGIN_SOUND,\
                    DES_OPP_ANGLE,\
                    FIN_SOUND,\
                    FRONT_ANGLE_MAX,\
                    FRONT_ANGLE_MIN,\
                    LEFT_TURN_ANGLE_MAX,\
                    LEFT_TURN_ANGLE_MIN,\
                    MACH_RADIUS,\
                    PWM_FREQ,\
                    PWM_PORTS,\
                    RIGHT_TURN_ANGLE_MAX,\
                    RIGHT_TURN_ANGLE_MIN,\
                    ROBOT,\
                    SENSOR,\
                    SLEEP_TIME,\
                    SNS_MAX_DISTANCE,\
                    SNS_MIN_DISTANCE,\
                    SNS_OPP_DISTANCE,\
                    SNS_RANGE,\
                    STNDBY_SOUND,\
                    START_TICK,\
                    STOP_TICK,\
                    STUCK_LEFT_TURN_MIN,\
                    STUCK_LEFT_TURN_MAX,\
                    STUCK_RIGHT_TURN_MIN,\
                    STUCK_RIGHT_TURN_MAX,\
                    TOTAL_SCANS,\
                    WHEELS

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

## ********************************************************
## name:      interpretData
## called by: lidarAnalysis.main()
## passed:    float[] sensorDistances, int lastFar,
##            int lastCW, int lastCCW
## returns:   int lastFar * tooFar + tooFar, 
##            int lastCW * turnCW + turnCW,
##            int lastCCW * turnCCW + turnCCW
## calls:     helperFunctions.getCollinearDistance()
##            repositionMachine.moveToOpponent()
##                              rotateMachine()
##
## Analyze sensor distance data and determine the         *
## appropriate course of action for maneuvering the       *
## machine if applicable.                                 *
## ********************************************************
def interpretData(distanceValues, lastFar, lastCW, lastCCW):

    #####################################
    ##
    ##	VARIABLE DECLARATION
    ##
    #####################################
                            
    activeDistances = []        ##Distances recorded to be too close too the 
                                ##machine; that is, a value recorded at any
                                ##angle that is less than MIN_DISTANCE.

    activeAngles = []           ##The corresponding angles associated with 
                                ##each activeDistances value.

    insertPoint = 0             ##The position in which an item is added to the
                                ##activeAngles and activeDistances lists.

    wallAnglesCount = 0         ##The amount of adjacent angles on either side
                                ##of the current value being added to 
                                ##activeAngles. These angles will also be added
                                ##to activeAngles if they are detecting the same
                                ##object as the original angle.

    adjustedDistance = 0        ##The returned value of getCollinearDistance().

    tooClose = False            ##Set to True if the machine detects an 
                                ##object that is too close.

    opponentSpan = 0            ##The amount of angles that have corresponding
                                ##distance values matching the location of the
                                ##opponent.
                                
    opponentAngle = 0           ##The average of all angles that are reading the
                                ##opponent.

    tooFar = False              ##Set to True if the machine detects the 
                                ##opponent is too far away.

    opponentFound = False       ##Set to True if the machine detects the 
                                ##opponent in front and tooClose == tooFar == 
                                ##False.

    turnMid = 0                 ##The midpoint of both the left and right turn
                                ##angles min/max values.

    turningCW = False           ##Set to True if the machine detects the 
                                ##opponent too far left of center.

    turningCCW = False          ##Set to True if the machine detects the 
                                ##opponent too far right of center. 

    canMoveForward = False      ##Set to True if the machine is able to detect
                                ##objects beyond MAX_DISTANCE. If none are
                                ##detected, allow for turning at any time.
    
    turnMin = turnMax = 0       ##The minimum and maximum angular values used to
                                ##determine if turning toward the opponent is
                                ##required. 

    minLast = 0                 ##The minimum value of the three function
                                ##parameters that keep track of how long an 
                                ##object has been tracked at a particular area.

    #####################################
    
    ##Collect position data for objects if considered too close to the machine.
    ##getCollinearDistance() is used iff there is first an object detected 
    ##within SNS_MIN_DISTANCE, and looks for other distance values that lie on 
    ##the same line as the original, out-of-range value. This will better allow 
    ##for detection of walls and other similar objects.
    for x in range (0, 360):

        if(MACH_RADIUS < distanceValues[x] < SNS_MIN_DISTANCE):
            insertPoint = bisect_left(activeAngles, x)

            if(len(activeAngles) == 0 or insertPoint == 
            len(activeAngles) or x != activeAngles[insertPoint]):
               activeAngles.insert(insertPoint, x)
               activeDistances.insert(insertPoint, distanceValues[x])
            else:
                continue
            
            wallAnglesCount = ceil(degrees(atan(MACH_RADIUS/distanceValues[x])))

            for y in range (x - wallAnglesCount, x + wallAnglesCount + 1):
                if(y < 0):
                    y += 360
                elif(y > 359): 
                    y -= 360

                adjustedDistance = getCollinearDistance(y, x, distanceValues[x])

                if(MACH_RADIUS < distanceValues[y] <= adjustedDistance):
                    
                    insertPoint = bisect_left(activeAngles, y)
                    
                    if(len(activeAngles) == 0 or insertPoint == 
                    len(activeAngles) or y != activeAngles[insertPoint]):
                        activeAngles.insert(insertPoint, y)
                        activeDistances.insert(insertPoint, distanceValues[y])

            tooClose = True

    if(tooClose):
        calculateObjMovement(activeAngles, activeDistances, distanceValues)
        return 0, 0, 0
    
    ##Barring any objects too close to the machine, look for the opponent being
    ##too far away. getCollinearDistance() is used here as to allow the opponent
    ##to move laterally when at MAX_DISTANCE, without the machine detecting them
    ##as too far.
    for x in range (FRONT_ANGLE_MIN, FRONT_ANGLE_MAX + 1):
        adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE, 
                                                SNS_MAX_DISTANCE)

        if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
            opponentFound = True

        if(distanceValues[x] > adjustedDistance):
            canMoveForward = True

        adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE, 
                                                        SNS_OPP_DISTANCE)

        if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
            tooFar = True

        if(opponentFound and tooFar and canMoveForward):
            break

    ##Analyze the potential need to turn toward the opponent in either
    ##direction.
    if(canMoveForward):
        turnMin = LEFT_TURN_ANGLE_MIN
        turnMax = LEFT_TURN_ANGLE_MAX
    else:
        turnMin = STUCK_LEFT_TURN_MIN
        turnMax = STUCK_LEFT_TURN_MAX

    turnMid = ceil((turnMin + turnMax) / 2)
        
    for x in range (turnMin, turnMax + 1):

        adjustedDistance = getCollinearDistance(x, turnMid,
                                                SNS_MAX_DISTANCE)

        if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
            turningCW = True
            break
            
    if(canMoveForward):
        turnMin = RIGHT_TURN_ANGLE_MIN
        turnMax = RIGHT_TURN_ANGLE_MAX
    else:
        turnMin = STUCK_RIGHT_TURN_MIN
        turnMax = STUCK_RIGHT_TURN_MAX

    turnMid = ceil((turnMin + turnMax) / 2)

    for x in range (turnMin, turnMax + 1):
            
        adjustedDistance = getCollinearDistance(x, turnMid, 
                                                SNS_MAX_DISTANCE)
                
        if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
            turningCCW = True
            break

    ##Account for objects that could be mistaken for the opponent by choosing to
    ##move in the direction where an object has been seen for the least amount 
    ##of time. This will most likely be the opponent repositioning themselves.
    if(not opponentFound):
        lastFar = -1 if (not tooFar) else lastFar
        lastCW = -1 if (not turningCW) else lastCW
        lastCCW = -1 if (not turningCCW) else lastCCW

        minLast = min([x for x in [lastFar, lastCW, lastCCW] if x >= 0], 
                      default=0)

        if(tooFar and minLast == lastFar):
            moveToOpponent()
        elif(turningCW and minLast == lastCW):
            rotateMachine(True, FRONT_ANGLE_MIN, FRONT_ANGLE_MAX)
        elif(turningCCW and minLast == lastCCW):
            rotateMachine(False, FRONT_ANGLE_MIN, FRONT_ANGLE_MAX)
        return 0, 0, 0

    elif (not canMoveForward):
        if(turningCW):
            rotateMachine(True, STUCK_RIGHT_TURN_MIN, STUCK_RIGHT_TURN_MAX)
        elif(turningCCW):
            rotateMachine(False, STUCK_LEFT_TURN_MIN, STUCK_LEFT_TURN_MAX)
        return 0, 0, 0

    return (lastFar * tooFar + tooFar, 
            lastCW * turningCW + turningCW, 
            lastCCW * turningCCW + turningCCW)

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

def run_robot(ROBOT):

    #####################################
    ##
    ##	VARIABLE DECLARATION
    ##
    #####################################

    sensorDistances = []                ##A list of all measured sensor
                                        ##distances for each angle, 0-359.

    lastFar = lastCW = lastCCW = 0      ##Keep track of the position statuses
                                        ##from the last iteration of analyzing
                                        ##sensor data.

    #####################################
    
    # get the time step of the current world.
    timestep = 32 #int(ROBOT.getBasicTimeStep())
    
    for i in range (3):
        WHEELS[i].setPosition(float('inf'))
        WHEELS[i].setVelocity(0.0)
        
        
    SENSOR.enable(timestep)    
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while ROBOT.step(timestep) != -1:
        sensorDistances = SENSOR.getRangeImage()[::-1]
        for i in range(360):
            sensorDistances[i] *= 39.3701
        
        lastFar, lastCW, lastCCW = interpretData(sensorDistances, lastFar, 
                                                 lastCW, lastCCW)
        
if __name__ == "__main__":
# create the Robot instance.
    run_robot(ROBOT)