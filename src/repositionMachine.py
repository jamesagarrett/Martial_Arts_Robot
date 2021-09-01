##  
##  James Garrett
##
##  repositionMachine.py
##  Last Updated: September 1, 2021
##
##  Perform a maneuver action of either: turning the machine, moving toward the
##  opponent, or moving away from an object, until back within the desired range
##  of the machine described in globals.py.
##

from math import atan, ceil, cos, degrees, floor, radians, sin 
from bisect import bisect_left  
from time import sleep

##Module that includes functions that are used throughout the project.
from helperFunctions import getCollinearDistance, getCartesianAngle,\
                            getPathAngles, playSoundEff

##THESE VALUES ARE NOT TO BE CHANGED!

from globals import ANGLE_ERR,\
                    CCW_COEFS,\
                    CW_COEFS,\
                    DES_OPP_ANGLE,\
                    FIN_SOUND,\
                    FRONT_ANGLE_MIN,\
                    FRONT_ANGLE_MAX,\
                    MACH_RADIUS,\
                    MAX_SPEED,\
                    PWM_FREQ,\
                    PWM_PORTS,\
                    SENSOR,\
                    SLEEP_TIME,\
                    SNS_MAX_DISTANCE,\
                    SNS_MIN_DISTANCE,\
                    SNS_OPT_DISTANCE,\
                    SNS_RANGE,\
                    SPEED_CONSTS,\
                    START_TICK,\
                    STOP_TICK,\
                    TURN_SPEED,\
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
## name:      calculatePWM
## called by: repositionMachine.moveFromObject()
##                              moveToOpponent()
##                              rotateMachine()
## passed:    int x, float abs(wheelSpeed * MAX_SPEED),
##            bool True/False
## returns:   int pwm
## calls:     nothing
##
## Calculates the appropriate PWM output value based on   *
## the desired RPM value for the specified wheel and      *
## direction.                                             *      
## ********************************************************
def calculatePWM(wheel, rpm, spinCW):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    pwm = 0     ##The corresponding PWM value for the given wheel RPM.
    
    #####################################

    if(spinCW):
        pwm = CW_COEFS[wheel][0] * rpm**2 \
            + CW_COEFS[wheel][1] * rpm \
            + CW_COEFS[wheel][2]
    else:
        pwm = CCW_COEFS[wheel][0] * rpm**2 \
            + CCW_COEFS[wheel][1] * rpm \
            + CCW_COEFS[wheel][2]
    
    return round(pwm)

## ********************************************************
## name:      moveFromObject
## called by: maneuverAnalysis.calculateObjMovement()
## passed:    int maneuverAngle, int[] pathAngles,
##            int newObjAngle, int desiredDistance,
##            float[] allDistances
## returns:   nothing
## calls:     repositionMachine.calculatePWM()
##            helperFunctions.getCartesianAngle()
##                            playSoundEff()
##
## Repositions the machine away from any object considered*
## too close, at the specified angular location and       *     
## distance.                                              *        
## ********************************************************
def moveFromObject(repositionAngle, watchAngles, targetAngle, desiredDistance,
                   allDistances):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################
    
    stopAngles = [targetAngle]     ##The angle values that the machine will look
                                   ##at to assess if it has moved the correct
                                   ##distance away from the object that was 
                                   ##deemed too close. Multiple values are used
                                   ##as a failsafe in case the targetAngle value
                                   ##does not return a distance.

    currDistances = []             ##The current distance values for the
                                   ##stopAngles values before repositioning and
                                   ##updated as repositioning occurs.

    lookForShorter = False         ##Set to True if majority of currDistances
                                   ##values are already greater than the
                                   ##desiredDistance value.

    wheelSpeeds = [0]*3            ##The speed for each wheel in order to
                                   ##reposition at the desired angle; this
                                   ##value will act as a percentage, values
                                   ##greater than 0 mean ccw wheel rotation,
                                   ##less than zero, cw wheel rotation.

    speedVars = [0]*2              ##Variables used to determine wheelSpeeds
                                   ##values.

    wheelPWMs = [0]*3              ##The PWM output values for each wheel.

    doneMoving = False             ##Set to True if the new desired position is
                                   ##reached, or an object is detected too
                                   ##close to the machine.

    adjustedDistance = 0           ##The returned value of 
                                   ##getCollinearDistance().

    index = 0                      ##The index of a given list in which a value
                                   ##is stored if present.
    
    #####################################

    for x in range(1, ANGLE_ERR + 1):
        stopAngles.insert(0, targetAngle - x)
        if(stopAngles[0] < 0):
            stopAngles[0] += 360

        stopAngles.append(targetAngle + x)
        if(stopAngles[len(stopAngles)-1] > 359):
            stopAngles[len(stopAngles)-1] -= 360

    stopAngles.sort()
    currDistances = [allDistances[x] for x in stopAngles]
    
    if(sum(x > desiredDistance for x in currDistances) > ANGLE_ERR):
        lookForShorter = True

    ##See documentation for explanation on how the following equations were 
    ##determined.

    speedVars[0] = sin(radians(repositionAngle))

    speedVars[1] = cos(radians(repositionAngle))

    wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
                     - speedVars[0]*SPEED_CONSTS[2]) \
                     / (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
                     - SPEED_CONSTS[1]*SPEED_CONSTS[2]) 

    wheelSpeeds[1] = (speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) \
                     / SPEED_CONSTS[3]

    wheelSpeeds[2] = -wheelSpeeds[1] - wheelSpeeds[0]
    
    for x in range (3):
        if(wheelSpeeds[x] == 0):
            wheelPWMs[x] = STOP_TICK 
        elif(wheelSpeeds[x] > 0):
            wheelPWMs[x] = calculatePWM(x, wheelSpeeds[x] * MAX_SPEED, False)
        else:
            wheelPWMs[x] = calculatePWM(x, abs(wheelSpeeds[x] * MAX_SPEED), 
                                        True)
    
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

                index = bisect_left(stopAngles, angle)

                if(index < len(stopAngles) and angle == stopAngles[index]):
                    if(distance == 0.0):
                        distance = SNS_RANGE 
                    currDistances[index] = distance

                    if(lookForShorter):
                        if(sum(x <= desiredDistance for x in currDistances) > 
                        ANGLE_ERR):
                            doneMoving = True
                            break
                    else:
                        if(sum(x >= desiredDistance for x in currDistances) > 
                        ANGLE_ERR):
                            doneMoving = True
                            break

            if(doneMoving):
                WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
                WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
                WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
                break

        ##Prevents adafruit_rplidar.py runtime error when attempting to collect
        ##data again
        SENSOR.stop()
        SENSOR.disconnect()
        SENSOR.connect()

    except:
        print("TERMINATING")
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
        playSoundEff(FIN_SOUND)
        SENSOR.stop()
        SENSOR.disconnect()
        sleep(SLEEP_TIME)
        
    return

## ********************************************************
## name:      moveToOpponent
## called by: lidarAnalysis.interpretData()
## passed:    nothing
## returns:   nothing
## calls:     repositionMachine.calculatePWM()
##            helperFunctions.getCartesianAngle(),
##                            getPathAngles()
##                            playSoundEff()
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

    wheelPWMs = [0]*3               ##The PWM output values for each wheel.

    watchAngles = []                ##Angular values that are in the path of the
                                    ##machine as it's moving that need to be
                                    ##checked for objects being too close.

    doneMoving = False              ##Set to True if the new desired position is
                                    ##reached, or an object is detected too
                                    ##close to the machine.

    index = 0                       ##The index of a given list in which a value
                                    ##is stored if present.

    #####################################

    ##See documentation for explanation on how the following equations were 
    ##determined.

    speedVars[0] = sin(radians(DES_OPP_ANGLE)) 

    speedVars[1] = cos(radians(DES_OPP_ANGLE))

    wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
                     - speedVars[0]*SPEED_CONSTS[2]) \
                     / (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
                     - SPEED_CONSTS[1]*SPEED_CONSTS[2]) 

    wheelSpeeds[1] = (speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) \
                     / SPEED_CONSTS[3]

    wheelSpeeds[2] = -wheelSpeeds[1] - wheelSpeeds[0]

    for x in range (3):
        if(wheelSpeeds[x] == 0):
            wheelPWMs[x] = STOP_TICK
        elif(wheelSpeeds[x] > 0):
            wheelPWMs[x] = calculatePWM(x, wheelSpeeds[x]*MAX_SPEED, False)
        else:
            wheelPWMs[x] = calculatePWM(x, abs(wheelSpeeds[x]*MAX_SPEED), True)
 
    watchAngles = getPathAngles(DES_OPP_ANGLE)

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
                WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
                WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
                WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
                break 

        ##Prevents adafruit_rplidar.py runtime error when attempting to collect 
        ##data again
        SENSOR.stop()
        SENSOR.disconnect()
        SENSOR.connect()    

    except:
        print("TERMINATING")
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
        playSoundEff(FIN_SOUND)
        SENSOR.stop()
        SENSOR.disconnect()
        sleep(SLEEP_TIME)
        
    return

# ********************************************************
## name:      rotateMachine
## called by: lidarAnalysis.interpretData()
## passed:    bool True/False, int MIN area value,
##            int MAX area value
## returns:   nothing
## calls:     repositionMachine.calculatePWM()
##            helperFunctions.getCartesianAngle(),
##                            getCollinearDistance()
##                            playSoundEff()
##
## Rotate the machine to face the opponent.               *
## ********************************************************
def rotateMachine(turnCW, stopMin, stopMax):
    
    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    wheelPWMs = [0]*3               ##The PWM output values for each wheel.
    
    doneMoving = False              ##Set to True once the machine is again 
                                    ##facing the opponent, or an object is 
                                    ##detected too close to the machine.

    adjustedDistance = 0            ##The returned value of
                                    ##getCollinearDistance().

    index = 0                       ##The index of a given list in which a value
                                    ##is stored if present.

    #####################################

    for x in range (3):
        if(turnCW):
            wheelPWMs[x] = calculatePWM(x, TURN_SPEED, False)
        else:
            wheelPWMs[x] = calculatePWM(x, TURN_SPEED, True)

    try:
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, wheelPWMs[0])
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, wheelPWMs[1])
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, wheelPWMs[2])

        for scan in SENSOR.iter_scans():
        
            for (_, angle, distance) in scan:
                angle = getCartesianAngle(round(angle))
                distance *= 0.0393

                if(MACH_RADIUS < distance <= SNS_MIN_DISTANCE):
                    doneMoving = True
                    break
         
                adjustedDistance = getCollinearDistance(angle, DES_OPP_ANGLE, 
                                                        SNS_MAX_DISTANCE)

                if(stopMin <= angle <= stopMax and MACH_RADIUS 
                   < distance <= adjustedDistance):
                    doneMoving = True
                    break
    
            if(doneMoving):
                WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
                WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
                WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
                break  

        ##Prevents adafruit_rplidar.py runtime error when attempting to collect 
        ##data again
        SENSOR.stop()
        SENSOR.disconnect()
        SENSOR.connect()

    except:
        print("TERMINATING")
        WHEELS.set_pwm(PWM_PORTS[0], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[1], START_TICK, STOP_TICK)
        WHEELS.set_pwm(PWM_PORTS[2], START_TICK, STOP_TICK)
        playSoundEff(FIN_SOUND)
        SENSOR.stop()
        SENSOR.disconnect()
        sleep(SLEEP_TIME)
            
    return
