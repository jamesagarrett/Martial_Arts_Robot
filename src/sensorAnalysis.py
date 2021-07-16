##	
##  James Garrett
##
##  sensorAnalysis.py
##  Last Updated: July 14, 2021
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
from maneuverAnalysis import calculateObjMovement, calculateOppMovement

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
                    STOP_SPEED,\
                    TOTAL_SCANS, \
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
## returns:   nothing
## calls:     sensorAnalysis.interpretData()
##            helperFunctions.getCartesianAngle()
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
    
    distancesCount = 0          ##The amount of measured sensor distances from
                                ##the current iteration of the distance 
                                ##collection loop. 

    listStartPnt = 0            ##The position in sensorDistances to begin 
                                ##iterating when manually entering values; done
                                ##when the sensor cannot read a value by itself.
    
    #####################################
    
    for i, scan in enumerate(SENSOR.iter_scans(), start=1):

        ##Collect and store sensor data. Distances are stored in such a way
        ##that the corresponding index for each value represents its angle on
        ##the standard Cartesian plane. Due to the clockwise rotation of the 
        ##sensor, values are also initially read in this manner before being 
        ##adjusted here. Distances are converted from millimeters to inches.
        for (_, angle, distance) in scan:
            if(distance <= 0.0):
                continue

            cartAngle = getCartesianAngle(floor(angle))
            if(sensorDistances[cartAngle] > 0.0):
                cartAngle = getCartesianAngle(ceil(angle))
                if(sensorDistances[cartAngle] > 0.0):
                    continue

            distancesCount += 1
            sensorDistances[cartAngle] = distance * 0.0393

        if(i == TOTAL_SCANS or distancesCount == 360):
            break

    ##Prevents adafruit_rplidar.py runtime error when attempting to collect 
    ##data again
    SENSOR.stop()
    SENSOR.disconnect()
    SENSOR.connect()

    interpretData(sensorDistances)

## ********************************************************
## name:      interpretData
## called by: sensorAnalysis.collectData()
## passed:    float[] sensorDistances 
## returns:   nothing
## calls:     helperFunctions.getCollinearDistance()
##            repositionMachine.moveToOpponent()
##                              rotateMachine()
##
## Analyze sensor distance data and determine the         *
## appropriate course of action for maneuvering the       *
## machine if applicable.                                 *
## ********************************************************
def interpretData(distanceValues):

    #####################################
    ##
    ##	VARIABLE DECLARATION
    ##
    #####################################
                            
    activeDistances = []        ##Distances recorded to be outside the desired 
                                ##range of the machine; that is, a value in 
                                ##front of the machine > MAX_DISTANCE or a 
                                ##value recorded anywhere < MIN_DISTANCE.

    activeAngles = []           ##The corresponding angles associated with 
                                ##each activeDistances value, or in the case of
                                ##turning toward the opponent, the list of 
                                ##angles where the opponent is currently 
                                ##located.

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
        ##The opponent's location is needed to make sure that when moving away
        ##from objects that are currently too close, the machine is not in turn
        ##too far from the opponent after moving.
        for x in range (FRONT_ANGLE_MIN, FRONT_ANGLE_MAX + 1):
            adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE, 
                                                    SNS_MAX_DISTANCE)

            if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
                opponentAngle += x
                opponentSpan += 1
        
        if(opponentSpan > 0):
            opponentAngle = floor(opponentAngle/opponentSpan)
        else:
            opponentAngle = DES_OPP_ANGLE

        print("Status: Too Close\n")
        #calculateObjMovement(activeAngles, activeDistances, opponentAngle, 
        #					  distanceValues)

        return

    ##Barring any objects too close to the machine, look for the opponent being
    ##too far away. getCollinearDistance() is used here as to allow the opponent
    ##to move laterally when at MAX_DISTANCE, without the machine detecting them
    ##as too far.
    for x in range (FRONT_ANGLE_MIN, FRONT_ANGLE_MAX + 1):
        adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE, 
                                                SNS_MAX_DISTANCE)

        if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
            activeAngles.clear()
            activeDistances.clear()
            tooFar = False
            opponentFound = True
            break

        adjustedDistance = getCollinearDistance(x, DES_OPP_ANGLE,
                                                 SNS_OPP_DISTANCE)

        if(MACH_RADIUS < distanceValues[x] <= adjustedDistance):
            activeAngles.append(x)
            activeDistances.append(distanceValues[x])	
            tooFar = True	 

    if(tooFar):
        print("Status: Too Far\n")
        moveToOpponent()

        return

    ##Analyze turning only after determining nothing is too close or too far 
    ##from the machine.
    if(not opponentFound):
        turnMid = ceil((LEFT_TURN_ANGLE_MIN + LEFT_TURN_ANGLE_MAX) / 2)
        
        for x in range (LEFT_TURN_ANGLE_MIN, LEFT_TURN_ANGLE_MAX + 1):

            adjustedDistance = getCollinearDistance(x, turnMid,
                                                    SNS_MAX_DISTANCE)

            if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
                activeAngles.append(x)
                turningCCW = True
            
        if(not turningCCW):
            
            turnMid = ceil((RIGHT_TURN_ANGLE_MIN + RIGHT_TURN_ANGLE_MAX) / 2)

            for x in range (RIGHT_TURN_ANGLE_MIN, RIGHT_TURN_ANGLE_MAX + 1):
                
                adjustedDistance = getCollinearDistance(x, turnMid, 
                                                        SNS_MAX_DISTANCE)
                
                if(SNS_MIN_DISTANCE <= distanceValues[x] <= adjustedDistance):
                    activeAngles.append(x)
                    turningCW = True

        if(turningCCW or turningCW):
            print("Status: Not Centered")
            #print(activeAngles)
            rotateMachine(turningCW)
        else:
            print("Status: No Opponent\n")
    else:
        print("Status: Good\n")

    return

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

## ********************************************************
## name:      main
## called by: nobody
## passed:    nothing 
## returns:   nothing
## calls:     sensorAnalysis.collectData()
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
    
    #####################################
    
    input("PRESS <ENTER> TO BEGIN")
    WHEELS.set_pwm_freq(PWM_FREQ)
    
    try:
        while(True):
            if(reset % 10 == 0):
                clear()
                reset = 0
            reset += 1

            collectData()

    except KeyboardInterrupt:
        clear()
        print("TERMINATING")
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()

    return

main()

