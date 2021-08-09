##	
##  James Garrett
##
##  sensorAnalysis.py
##  Last Updated: August 9, 2021
##
##  Collect and analyze sensor distance data to determine whether repositioning 
##  is needed. If so, also determine in what manner the machine needs to 
##  maneuver itself to be back within the desired range described in globals.py.
##

from math import atan, ceil, degrees, floor
from bisect import bisect_left	
from os import system

##Module that includes functions that are used throughout the project.
from helperFunctions import getCollinearDistance, getCartesianAngle

##Module to determine how the machine will reposition itself toward or
##away from the opponent.
from maneuverAnalysis import calculateObjMovement

##Module for performing repositioning of the machine, based on sensor data 
##collection and analysis.
from repositionMachine import moveToOpponent, rotateMachine

##THESE VALUES ARE NOT TO BE CHANGED!
from globals import DES_OPP_ANGLE,\
                    FRONT_ANGLE_MAX,\
                    FRONT_ANGLE_MIN,\
                    LEFT_TURN_ANGLE_MAX,\
                    LEFT_TURN_ANGLE_MIN,\
                    MACH_RADIUS,\
                    PWM_FREQ,\
                    PWM_PORTS,\
                    RIGHT_TURN_ANGLE_MAX,\
                    RIGHT_TURN_ANGLE_MIN,\
                    SENSOR,\
                    SNS_MAX_DISTANCE,\
                    SNS_MIN_DISTANCE,\
                    SNS_OPP_DISTANCE,\
                    START_TICK,\
                    STOP_TICK,\
                    TOTAL_SCANS,\
                    WHEELS

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:      collectData
## called by: sensorAnalysis.main()
## passed:    nothing
## returns:   float[] sensorDistances
## calls:     helperFunctions.getCartesianAngle()
##
## Retrieve sensor readings and store all angles and	  *
## associated distances.                                  *
## ********************************************************
def collectData():

    #####################################
    ##
    ##	VARIABLE DECLARATION
    ##
    #####################################

    sensorDistances = [0.0]*360 ##A list of all measured sensor distances for 
                                ##each angle, 0-359.

    cartAngle = 0               ##The angle measurement returned from 
                                ##getCartesianAngle().
    
    #####################################
    
    for i, scan in enumerate(SENSOR.iter_scans(), start=1):

        ##Collect and store sensor data. Distances are stored in such a way
        ##that the corresponding index for each value represents its angle on
        ##the standard Cartesian plane. Due to the clockwise rotation of the 
        ##sensor, values are also read in clockwise before being adjusted here.
        ##Distances are converted from millimeters to inches.
        for (_, angle, distance) in scan:
            if(distance == 0.0):
                continue

            cartAngle = getCartesianAngle(floor(angle))
            if(sensorDistances[cartAngle] > 0.0):
                cartAngle = getCartesianAngle(ceil(angle))
                if(sensorDistances[cartAngle] > 0.0):
                    cartAngle = getCartesianAngle(round(angle))

            sensorDistances[cartAngle] = distance * 0.0393

        if(i == TOTAL_SCANS):
            break

    ##Prevents adafruit_rplidar.py runtime error when attempting to collect 
    ##data again
    SENSOR.stop()
    SENSOR.disconnect()
    SENSOR.connect()

    return sensorDistances

## ********************************************************
## name:      interpretData
## called by: sensorAnalysis.main()
## passed:    float[] sensorDistances, int lastFar,
##            int lastCCW, int lastCW
## returns:   int lastFar * tooFar + tooFar, 
##            int lastCCW * turnCCW + turnCCW,
##            int lastCW * turnCW + turnCW
## calls:     helperFunctions.getCollinearDistance()
##            repositionMachine.moveToOpponent()
##                              rotateMachine()
##
## Analyze sensor distance data and determine the         *
## appropriate course of action for maneuvering the       *
## machine if applicable.                                 *
## ********************************************************
def interpretData(distanceValues, lastFar, lastCCW, lastCW):

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
        #print("Status: Too Close\n")
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

        adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE,
                                                 SNS_OPP_DISTANCE)

        if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
            tooFar = True

        if(opponentFound and tooFar):
            break

    ##Analyze the potential need to turn toward the opponent in either
    ##direction.
    turnMid = ceil((LEFT_TURN_ANGLE_MIN + LEFT_TURN_ANGLE_MAX) / 2)
        
    for x in range (LEFT_TURN_ANGLE_MIN, LEFT_TURN_ANGLE_MAX + 1):

        adjustedDistance = getCollinearDistance(x, turnMid,
                                                SNS_MAX_DISTANCE)

        if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
            turningCCW = True
            break
            
    turnMid = ceil((RIGHT_TURN_ANGLE_MIN + RIGHT_TURN_ANGLE_MAX) / 2)

    for x in range (RIGHT_TURN_ANGLE_MIN, RIGHT_TURN_ANGLE_MAX + 1):
            
        adjustedDistance = getCollinearDistance(x, turnMid, 
                                                SNS_MAX_DISTANCE)
                
        if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
            turningCW = True
            break

    print(lastFar, tooFar, lastCCW, turnCCW, lastCW, turnCW)

    ##Account for objects that could be mistaken for the opponent by choosing to
    ##move in the direction where an object has been seen for the least amount 
    ##of time. This will most likely be the opponent repositioning themselves.
    if(not opponentFound):
        lastFar = None if (not tooFar) else lastFar
        lastCCW = None if (not turnCCW) else lastCCW
        lastCW = None if (not turnCW) else lastCW

        minLast = min(x for x in [lastFar, lastCCW, lastCW] if x is not None)

        if(turnCCW and minLast == lastCCW):
            print("Turn CCW")
            rotateMachine(True)
        elif(turnCW and minLast == lastCW):
            print("Turn CW")
            rotateMachine(False)
        elif(tooFar and minLast == lastFar):
            print("Move Forward")
            moveToOpponent()

        return 0, 0, 0

    return (lastFar * tooFar + tooFar, 
            lastCCW * turningCCW + turningCCW, 
            lastCW * turningCW + turningCW)

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

## ********************************************************
## name:      main
## called by: nobody
## passed:    nothing 
## returns:   nothing
## calls:     sensorAnalysis.collectData()
##                          .interpretData()
##
## Begins program when user is ready, clearing the        *
## console periodically of any print messages before      *
## terminating on keyboard interrupt.                     *
## ********************************************************
def main():
    #####################################
    ##
    ##	VARIABLE DECLARATION
    ##
    #####################################

    clear = lambda:system('clear')      ##Used for clearing the terminal screen.

    reset = 0                           ##Determines when to clear the terminal
                                        ##screen.
    
    sensorDistances = []                ##A list of all measured sensor
                                        ##distances for each angle, 0-359.

    lastFar = lastCCW = lastCW = 0      ##Keep track of the position statuses
                                        ##from the last iteration of analyzing
                                        ##sensor data.

    #####################################
    
    clear()
    input("PRESS <ENTER> TO BEGIN")
    WHEELS.set_pwm_freq(PWM_FREQ)
    
    try:
        while(True):
            if(reset % 100 == 0):
                clear()
                reset = 0
            reset += 1

            sensorDistances = collectData()
            lastFar, lastCCW, lastCW = interpretData(sensorDistances, lastFar, 
                                                     lastCCW, lastCW)

    except:
        #clear()
        print("TERMINATING")
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
        SENSOR.stop()
        SENSOR.disconnect()

    return

main()

