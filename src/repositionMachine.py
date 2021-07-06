##  
##  James Garrett
##
##  Martial_Arts_Robot 
##  Last Updated: July 6, 2021
##
##  repositionMachine.py
##  Last Updated: July 6, 2021
##
##  Perform a maneuver action of either: turning the machine, moving toward the
##  opponent, or moving away from an object, until back within the desired range
##  of the machine described in globals.py.
##

from math import atan, ceil, cos, degrees, floor, radians, round, sin 
from bisect import bisect_left  
from os import system

##Module that includes functions that are used throughout the project.
from helperFunctions import getCollinearDistance, getCartesianAngle

##THESE VALUES ARE NOT TO BE CHANGED!
from globals import DES_OPP_ANGLE,\
                    MACH_RADIUS,\
                    MAX_SPEED,\
                    MAX_CCW_SPEED,\
                    MAX_CW_SPEED,\
                    MIN_CCW_SPEED,\
                    MIN_CW_SPEED,\
                    PWM_FREQ,\
                    PWM_PORTS,\
                    SENSOR,\
                    SNS_MAX_DISTANCE,\
                    SNS_MIN_DISTANCE,\
                    SNS_OPT_DISTANCE,\
                    SPEED_CONSTS,\
                    START_TICK,\
                    STOP_SPEED,\
                    WHEELS,\
                    WHEEL_DIRECTIONS 

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

clear = lambda: system('clear')

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:      moveFromObject
## called by: maneuverAnalysis.calculateObjMovement()
## passed:    int maneuverAngle, float maneuverDistance,
##            float moveObjDistance, int[] pathAngles, 
##            float MACH_RADIUS
## returns:   nothing
## calls:     helperFunctions.getCartesianAngle()
##
## Repositions the machine away from any object considered*
## too close, at the specified angular location and       *     
## distance.                                              *        
## ********************************************************
def moveFromObject(repositionAngle, repositionDistance, objectDistance,        
                   watchAngles, xBound):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################
    
    stopAngles = [repositionAngle]  ##The angles in which the machine will look
                                    ##to stop moving once at repositionDistance;
                                    ##repositionAngle is the center-point for 
                                    ##the movement path, and thus the preferred
                                    ##angle used, however multiple values are
                                    ##used as a failsafe in case an angle
                                    ##doesn't return a distance value.

    stopDistances = \
        [repositionDistance]        ##The corresponding distances associated 
                                    ##with each stopAngles value.

    angleBound = 0                  ##The angle with respect to repositionAngle
                                    ##in which the x-coordinate boundary - 
                                    ##xBound - and y-coordinate boundary - 
                                    ##repositionDistance - intersect. Angles 
                                    ##between repositionAngle and angleBound are
                                    ##added to stopAngles.

    wheelSpeeds = [0]*3             ##The speed for each wheel in order to
                                    ##reposition at the desired angle; this
                                    ##value will act as a percentage, values
                                    ##greater than 0 mean ccw wheel rotation,
                                    ##less than zero, cw wheel rotation.

    speedVars = [0]*2               ##Variables used to determine wheelSpeeds
                                    ##values.

    rotationCoeff = 0               ##Value used to change wheelSpeeds values to
                                    ##allow for machine rotation in order to
                                    ##face the opponent while repositioning; set
                                    ##to 0 when only linear movement is desired;
                                    ##this value will act as a percentage,
                                    ##values greater than 0 mean ccw machine
                                    ##rotation, less than zero, cw rotation.

    wheelPWMs = [0]*3               ##The pwm output values for each wheel.

    doneMoving = False              ##Set to True if the new desired position is
                                    ##reached, or an object is detected too
                                    ##close to the machine.

    adjustedDistance = 0            ##The returned value of 
                                    ##getCollinearDistance().

    index = 0                       ##The index of a given list in which a value
                                    ##is stored if present.
    
    #####################################

    ##See documentation for explanation on how the following equations were 
    ##determined.

    angleBound = floor(degrees(atan(xBound/repositionDistance))) 

    for x in range(1, angleBound + 1):
        stopAngles.insert(0, watchAngles[len(watchAngles)//2 - x])
        stopAngles.append(watchAngles[len(watchAngles)//2 + x])

        stopDistances.insert(0, repositionDistance/cos(radians(x)))
        stopDistances.append(repositionDistance/cos(radians(x)))

    speedVars[0] = sin(radians(repositionAngle)) \
                   - rotationCoeff*sin(radians(WHEEL_DIRECTIONS[2]))

    speedVars[1] = cos(radians(repositionAngle)) \
                   - rotationCoeff*cos(radians(WHEEL_DIRECTIONS[2]))

    wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
                     - speedVars[0]*SPEED_CONSTS[2]) \
                     / (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
                     - SPEED_CONSTS[1]*SPEED_CONSTS[2]) 

    wheelSpeeds[1] = (speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) \
                     / SPEED_CONSTS[3]

    wheelSpeeds[2] = (-wheelSpeeds[1] - wheelSpeeds[0] + rotationCoeff)
    
    for x in range (0, len(PWM_PORTS)):
        if(wheelSpeeds[x] == 0):
            wheelPWMs[x] = STOP_SPEED 
        elif(wheelSpeeds[x] > 0):
            wheelPWMs[x] = floor(MIN_CCW_SPEED + MAX_SPEED/(1/wheelSpeeds[x]))
        else:
            wheelPWMs[x] = floor(MIN_CW_SPEED + MAX_SPEED/(1/wheelSpeeds[x]))
    
    print("Wheels: ", wheelSpeeds, wheelPWMs, "\n")

    try:
        WHEELS.set_pwm_freq(PWM_FREQ)
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, wheelPWMs[0])
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, wheelPWMs[1])
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, wheelPWMs[2])

        for scan in SENSOR.iter_scans():

            for (_, angle, distance) in scan:

                angle = getCartesianAngle(round(angle))
                distance *= 0.0393

                index = bisect_left(watchAngles, angle)

                if(index < len(watchAngles) and angle == watchAngles[index] and
                  MACH_RADIUS < distance <= SNS_MIN_DISTANCE):
                    doneMoving = True
                    break

                index = bisect_left(stopAngles, angle)

                if(index < len(stopAngles) and angle == stopAngles[index] and
                  MACH_RADIUS < distance <= objectDistance - stopDistances[index]):
                    doneMoving = True
                    break

            if(doneMoving):
                WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
                WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
                WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
                break

        ##Prevents adafruit_rplidar.py runtime error when attempting to collect
	    ##data again
        SENSOR.stop()
        SENSOR.disconnect()
        SENSOR.connect()

    except KeyboardInterrupt:
        clear()
        print("TERMINATING")
        WHEELS.set_pwm_freq(PWM_FREQ)
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()
            
    return

## ********************************************************
## name:      moveToOpponent
## called by: maneuverAnalysis.calculateOppMovement()
## passed:    int maneuverAngle, int[] pathAngles, 
##            int[] oppAngles
## returns:   nothing
## calls:     helperFunctions.getCartesianAngle()
##
## Repositions the machine towards the opponent at the    *
## specified angle location until reaching OPT_DISTANCE   *
## away.                                                  *
## ********************************************************
def moveToOpponent(repositionAngle, watchAngles, stopAngles):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    wheelSpeeds = [0]*3     ##The speed for each wheel in order to reposition 
                            ##at the desired angle; this value will act as a 
                            ##percentage, values greater than 0 mean ccw wheel 
                            ##rotation, less than zero, cw wheel rotation.

    speedVars = [0]*2       ##Variables used to determine wheelSpeeds values.

    rotationCoeff = 0       ##Value used to change wheelSpeeds values to allow 
                            ##for machine rotation in order to face the 
                            ##opponent while repositioning; set to 0 when only 
                            ##linear movement is desired; this value will act 
                            ##as a percentage, values greater than 0 mean ccw 
                            ##machine rotation,less than zero, cw rotation.

    wheelPWMs = [0]*3       ##The pwm output values for each wheel.

    doneMoving = False      ##Set to True if the new desired position is 
                            ##reached, or an object is detected too close to the
                            ##machine.

    index = 0               ##The index of a given list in which a value is
                            ##stored if present.

    #####################################

    ##See documentation for explanation on how the following equations were 
    ##determined.

    speedVars[0] = sin(radians(repositionAngle)) \
                   - rotationCoeff*sin(radians(WHEEL_DIRECTIONS[2]))

    speedVars[1] = cos(radians(repositionAngle)) \
                   - rotationCoeff*cos(radians(WHEEL_DIRECTIONS[2]))

    wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
                     - speedVars[0]*SPEED_CONSTS[2]) \
                     / (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
                     - SPEED_CONSTS[1]*SPEED_CONSTS[2]) 

    wheelSpeeds[1] = (speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) \
                     / SPEED_CONSTS[3]

    wheelSpeeds[2] = (-wheelSpeeds[1] - wheelSpeeds[0] + rotationCoeff)

    for x in range (0, len(PWM_PORTS)):
        if(wheelSpeeds[x] == 0):
            wheelPWMs[x] = STOP_SPEED 
        elif(wheelSpeeds[x] > 0):
            wheelPWMs[x] = floor(MIN_CCW_SPEED + MAX_SPEED/(1/wheelSpeeds[x]))
        else:
            wheelPWMs[x] = floor(MIN_CW_SPEED + MAX_SPEED/(1/wheelSpeeds[x]))
    
    print("Wheels: ", wheelSpeeds, wheelPWMs, "\n")
    
    try:
        WHEELS.set_pwm_freq(PWM_FREQ)
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, wheelPWMs[0])
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, wheelPWMs[1])
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, wheelPWMs[2])

        #for scan in SENSOR.iter_scans():
        
        #    for (_, angle, distance) in scan:
        #        angle = getCartesianAngle(round(angle))
        #        distance *= 0.0393

        #        index = bisect_left(watchAngles, angle)

        #        if(index < len(watchAngles) and angle == watchAngles[index] and
        #          MACH_RADIUS < distance <= SNS_MIN_DISTANCE):
        #            doneMoving = True
        #            break

        #        index = bisect_left(stopAngles, angle)

        #        if(index < len(stopAngles) and angle == stopAngles[index] and
        #          MACH_RADIUS < distance <= SNS_OPT_DISTANCE):
        #            doneMoving = True
        #            break

        #    if(doneMoving):
        #        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        #        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        #        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        #        break 

        ##Prevents adafruit_rplidar.py runtime error when attempting to collect 
	    ##data again
        SENSOR.stop()
        SENSOR.disconnect()
        SENSOR.connect()    

    except KeyboardInterrupt:
        clear()
        print("TERMINATING")
        WHEELS.set_pwm_freq(PWM_FREQ)
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()

    return

# ********************************************************
## name:      rotateMachine
## called by: sensorAnalysis.interpretData()
## passed:    int turningDirection, int len(activeAngles)
## returns:   nothing
## calls:     helperFunctions.getCartesianAngle(),
##                            getCollinearDistance()
##
## Rotate the machine to face the opponent.               *
## ********************************************************
def rotateMachine(turnCW, opponentSpan):
    
    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    stopAngles = [DES_OPP_ANGLE]    ##The angles in which the machine will look 
                                    ##to stop moving once facing the opponent; 
                                    ##DES_OPP_ANGLE is the center-point for the 
                                    ##opponent, and thus the preferred stopping 
                                    ##point, however multiple values are used as
                                    ##a failsafe in case an angle doesn't 
                                    ##return a distance value.

    doneMoving = False              ##Set to True once the machine is again 
                                    ##facing the opponent, or an object is 
                                    ##detected too close to the machine.

    adjustedDistance = 0            ##The returned value of
                                    ##getCollinearDistance().

    index = 0                       ##The index of a given list in which a value
                                    ##is stored if present.

    #####################################

    if(turnCW):
        print("Maneuver: Direction - Clockwise\n")
    else:
        print("Maneuver: Direction - Counter-Clockwise\n")

    for x in range (1, ceil(opponentSpan/2) + 1):
        stopAngles.insert(0, DES_OPP_ANGLE-x)
        stopAngles.append(DES_OPP_ANGLE+x)

    try:
        WHEELS.set_pwm_freq(PWM_FREQ)
        if(turnCW):
            WHEELS.set_pwm(PWM_PORTS[0], START_TICK, MAX_CW_SPEED)
            WHEELS.set_pwm(PWM_PORTS[1], START_TICK, MAX_CW_SPEED)
            WHEELS.set_pwm(PWM_PORTS[2], START_TICK, MAX_CW_SPEED)
        else:
            WHEELS.set_pwm(PWM_PORTS[0], START_TICK, MAX_CCW_SPEED)
            WHEELS.set_pwm(PWM_PORTS[1], START_TICK, MAX_CCW_SPEED)
            WHEELS.set_pwm(PWM_PORTS[2], START_TICK, MAX_CCW_SPEED) 

        for scan in SENSOR.iter_scans():
        
            for (_, angle, distance) in scan:
                angle = getCartesianAngle(round(angle))
                distance *= 0.0393

                if(MACH_RADIUS < distance <= SNS_MIN_DISTANCE):
                    doneMoving = True
                    break
         
                adjustedDistance = getCollinearDistance(angle, DES_OPP_ANGLE, 
                                                        SNS_MAX_DISTANCE)

                index = bisect_left(stopAngles, angle)

                if(index < len(stopAngles) and angle == stopAngles[index] and
                  MACH_RADIUS < distance <= adjustedDistance):
                    doneMoving = True
                    break
    
            if(doneMoving):
                WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
                WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
                WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
                break  

        ##Prevents adafruit_rplidar.py runtime error when attempting to collect 
	    ##data again
        SENSOR.stop()
        SENSOR.disconnect()
        SENSOR.connect()

    except KeyboardInterrupt:
        clear()
        print("TERMINATING")
        WHEELS.set_pwm_freq(PWM_FREQ)
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()

    return
