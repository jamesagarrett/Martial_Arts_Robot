##  
##  James Garrett
##
##  Martial_Arts_Robot 
##  Last Updated: July 14, 2021
##
##  repositionMachine.py
##  Last Updated: July 14, 2021
##
##  Perform a maneuver action of either: turning the machine, moving toward the
##  opponent, or moving away from an object, until back within the desired range
##  of the machine described in globals.py.
##

from math import atan, ceil, cos, degrees, floor, radians, sin 
from bisect import bisect_left  
from os import system

##Module that includes functions that are used throughout the project.
from helperFunctions import getCollinearDistance, getCartesianAngle

##Module to determine how the machine will reposition itself toward or
##away from the opponent.
from maneuverAnalysis import getPathAngles

##THESE VALUES ARE NOT TO BE CHANGED!

from globals import DES_OPP_ANGLE,\
                    FRONT_ANGLE_MIN,\
                    FRONT_ANGLE_MAX,\
                    MACH_RADIUS,\
                    MAX_SPEED,\
                    MAX_CCW_PWM,\
                    MAX_CW_PWM,\
                    MIN_CCW_PWM,\
                    MIN_CW_PWM,\
                    PWM_FREQ,\
                    PWM_PORTS,\
                    SENSOR,\
                    SNS_MAX_DISTANCE,\
                    SNS_MIN_DISTANCE,\
                    SNS_OPT_DISTANCE,\
                    SPEED_BOOST,\
                    SPEED_CONSTS,\
                    START_TICK,\
                    STOP_SPEED,\
                    WHEELS,\
                    WHEEL_DIRECTIONS 

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:      moveFromObject
## called by: maneuverAnalysis.calculateObjMovement()
## passed:    int maneuverAngle, float maneuverDistance,
##            float moveObjDistance, int[] pathAngles 
## returns:   nothing
## calls:     helperFunctions.getCartesianAngle()
##
## Repositions the machine away from any object considered*
## too close, at the specified angular location and       *     
## distance.                                              *        
## ********************************************************
def moveFromObject(repositionAngle, repositionDistance, objectDistance,        
                   watchAngles):

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
        [objectDistance - \
        repositionDistance]        ##The corresponding distances associated 
                                   ##with each stopAngles value. The machine has
                                   ##moved appropriately once these distance
                                   ##values have been reached.

    stopDist = 0                   ##The current value being added to 
                                   ##stopDistances.

    objectDistances = \
        [objectDistance]           ##The distance of the object being moved 
                                   ##toward for each stopAngles value.

    angleBound = 0                 ##The angle with respect to repositionAngle
                                   ##in which the x-coordinate boundary - 
                                   ##MACH_RADIUS - and y-coordinate boundary - 
                                   ##repositionDistance - intersect. Angles 
                                   ##between repositionAngle and angleBound are
                                   ##added to stopAngles.

    wheelSpeeds = [0]*3            ##The speed for each wheel in order to
                                   ##reposition at the desired angle; this
                                   ##value will act as a percentage, values
                                   ##greater than 0 mean ccw wheel rotation,
                                   ##less than zero, cw wheel rotation.

    speedVars = [0]*2              ##Variables used to determine wheelSpeeds
                                   ##values.

    rotationCoeff = 0              ##Value used to change wheelSpeeds values to
                                   ##allow for machine rotation in order to
                                   ##face the opponent while repositioning; set
                                   ##to 0 when only linear movement is desired;
                                   ##this value will act as a percentage,
                                   ##values greater than 0 mean ccw machine
                                   ##rotation, less than zero, cw rotation.

    wheelPWMs = [0]*3              ##The pwm output values for each wheel.

    doneMoving = False             ##Set to True if the new desired position is
                                   ##reached, or an object is detected too
                                   ##close to the machine.

    adjustedDistance = 0           ##The returned value of 
                                   ##getCollinearDistance().

    index = 0                      ##The index of a given list in which a value
                                   ##is stored if present.
    
    clear = lambda:system('clear') ##Used for clearing the terminal screen.

    #####################################

    ##See documentation for explanation on how the following equations were 
    ##determined.

    angleBound = floor(degrees(atan(MACH_RADIUS/repositionDistance))) 

    for x in range(1, angleBound + 1):
        stopAngles.insert(0, watchAngles[len(watchAngles)//2 - x])
        stopAngles.append(watchAngles[len(watchAngles)//2 + x])

        stopDist = objectDistance/cos(radians(x)) - \
                   repositionDistance/cos(radians(x))

        stopDistances.insert(0, stopDist)
        stopDistances.append(stopDist)

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
    
    for x in range (3):
        if(wheelSpeeds[x] == 0):
            wheelPWMs[x] = STOP_SPEED 
        elif(wheelSpeeds[x] > 0):
            wheelPWMs[x] = round(MIN_CCW_PWM + MAX_SPEED * wheelSpeeds[x])
            if(x == 0):
                wheelPWMs[x] += SPEED_BOOST
        else:
            wheelPWMs[x] = round(MIN_CW_PWM + MAX_SPEED * wheelSpeeds[x])
            if(x == 0):
                wheelPWMs[x] -= SPEED_BOOST

    #print("Wheels: ", wheelSpeeds, wheelPWMs, "\n")
    #print("Watch:\n", watchAngles, "\n\nStop:\n", stopAngles)

    try:
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, wheelPWMs[0])
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, wheelPWMs[1])
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, wheelPWMs[2])

        for scan in SENSOR.iter_scans():

            for (_, angle, distance) in scan:

                angle = getCartesianAngle(round(angle))
                distance *= 0.0393

                index = bisect_left(watchAngles, angle)

                if(index < len(watchAngles) and angle == watchAngles[index]):
                    if(MACH_RADIUS < distance < SNS_MIN_DISTANCE):
                        doneMoving = True
                        break
                else:
                    continue

                index = bisect_left(stopAngles, angle)

                if(MACH_RADIUS < distance <= stopDistances[index]):
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
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()
            
    return

## ********************************************************
## name:      moveToOpponent
## called by: sensorAnalysis.interpretData()
## passed:    nothing
## returns:   nothing
## calls:     helperFunctions.getCartesianAngle()
##            maneuverAnalysis.getPathAngles()
##
## Repositions the machine towards the opponent at the    *
## DES_OPP_ANGLE location until reaching OPT_DISTANCE     *
## away.                                                  *
## ********************************************************
def moveToOpponent():

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    wheelSpeeds = [0]*3             ##The speed for each wheel in order to
                                    ##reposition at the desired angle; this 
                                    ###value will act as a percentage, values
                                    ###greater than 0 mean ccw rotation, less 
                                    ##than zero, cw wheel rotation.

    speedVars = [0]*2               ##Variables used to determine wheelSpeeds 
                                    ##values.

    rotationCoeff = 0               ##Value used to change wheelSpeeds values to
                                    ##allow for machine rotation in order to
                                    ###face the opponent while repositioning;
                                    ###set to 0 when only linear movement is
                                    ##desired. This value will act as a
                                    ##percentage; values greater than 0 mean ccw
                                    ##machine rotation, less than zero, cw 
                                    ##rotation.

    wheelPWMs = [0]*3               ##The pwm output values for each wheel.

    watchAngles = []                ##Angular values that are in the path of the
                                    ##machine as it's moving that need to be
                                    ##checked for objects being too close.

    doneMoving = False              ##Set to True if the new desired position is
                                    ##reached, or an object is detected too
                                    ##close to the machine.

    index = 0                       ##The index of a given list in which a value
                                    ##is stored if present.

    clear = lambda:system('clear')  ##Used for clearing the terminal screen.

    #####################################

    ##See documentation for explanation on how the following equations were 
    ##determined.

    speedVars[0] = sin(radians(DES_OPP_ANGLE)) \
                   - rotationCoeff*sin(radians(WHEEL_DIRECTIONS[2]))

    speedVars[1] = cos(radians(DES_OPP_ANGLE)) \
                   - rotationCoeff*cos(radians(WHEEL_DIRECTIONS[2]))

    wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
                     - speedVars[0]*SPEED_CONSTS[2]) \
                     / (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
                     - SPEED_CONSTS[1]*SPEED_CONSTS[2]) 

    wheelSpeeds[1] = (speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) \
                     / SPEED_CONSTS[3]

    wheelSpeeds[2] = (-wheelSpeeds[1] - wheelSpeeds[0] + rotationCoeff)

    for x in range (len(WHEELS)):
        if(wheelSpeeds[x] == 0):
            wheelPWMs[x] = STOP_SPEED 
        elif(wheelSpeeds[x] > 0):
            wheelPWMs[x] = round(MIN_CCW_PWM + MAX_SPEED * wheelSpeeds[x])
            if(x == 0):
                wheelPWMs[x] += SPEED_BOOST
        else:
            wheelPWMs[x] = round(MIN_CW_PWM + MAX_SPEED * wheelSpeeds[x])
            if(x == 0):
                wheelPWMs[x] -= SPEED_BOOST
 
    watchAngles = getPathAngles(DES_OPP_ANGLE)

    print("Wheels: ", wheelSpeeds, wheelPWMs, "\n")
    #print("Watch:\n", watchAngles)

    try:
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, wheelPWMs[0])
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, wheelPWMs[1])
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, wheelPWMs[2])

        for scan in SENSOR.iter_scans():
        
            for (_, angle, distance) in scan:
                angle = getCartesianAngle(round(angle))
                distance *= 0.0393

                index = bisect_left(watchAngles, angle)

                if(index < len(watchAngles) and angle == watchAngles[index]):
                    if(MACH_RADIUS < distance < SNS_MIN_DISTANCE):
                        doneMoving = True
                        break
                else:
                    continue

                if(FRONT_ANGLE_MIN <= angle <= FRONT_ANGLE_MAX and
                   MACH_RADIUS < distance <= SNS_OPT_DISTANCE):
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
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()

    return

# ********************************************************
## name:      rotateMachine
## called by: sensorAnalysis.interpretData()
## passed:    int turningDirection
## returns:   nothing
## calls:     helperFunctions.getCartesianAngle(),
##                            getCollinearDistance()
##
## Rotate the machine to face the opponent.               *
## ********************************************************
def rotateMachine(turnCW):
    
    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    doneMoving = False              ##Set to True once the machine is again 
                                    ##facing the opponent, or an object is 
                                    ##detected too close to the machine.

    adjustedDistance = 0            ##The returned value of
                                    ##getCollinearDistance().

    index = 0                       ##The index of a given list in which a value
                                    ##is stored if present.

    clear = lambda:system('clear')  ##Used for clearing the terminal screen.

    #####################################

    #if(turnCW):
        #print("Maneuver: Direction - Clockwise\n")
    #else:
        #print("Maneuver: Direction - Counter-Clockwise\n")

    try:
        if(turnCW):
            WHEELS.set_pwm(PWM_PORTS[0], START_TICK, MAX_CW_PWM)
            WHEELS.set_pwm(PWM_PORTS[1], START_TICK, MAX_CW_PWM)
            WHEELS.set_pwm(PWM_PORTS[2], START_TICK, MAX_CW_PWM)
        else:
            WHEELS.set_pwm(PWM_PORTS[0], START_TICK, MAX_CCW_PWM)
            WHEELS.set_pwm(PWM_PORTS[1], START_TICK, MAX_CCW_PWM)
            WHEELS.set_pwm(PWM_PORTS[2], START_TICK, MAX_CCW_PWM) 

        for scan in SENSOR.iter_scans():
        
            for (_, angle, distance) in scan:
                angle = getCartesianAngle(round(angle))
                distance *= 0.0393

                if(MACH_RADIUS < distance <= SNS_MIN_DISTANCE):
                    doneMoving = True
                    break
         
                adjustedDistance = getCollinearDistance(angle, DES_OPP_ANGLE, 
                                                        SNS_MAX_DISTANCE)

                if(FRONT_ANGLE_MIN <= angle <= FRONT_ANGLE_MAX and
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
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_SPEED)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_SPEED)
        SENSOR.stop()
        SENSOR.disconnect()

    return
