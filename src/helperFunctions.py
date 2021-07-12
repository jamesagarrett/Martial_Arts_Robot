##  
##  James Garrett
##
##  Martial_Arts_Robot 
##  Last Updated: July 11, 2021
##
##  helperFunctions.py
##  Last Updated: July 11, 2021
##
##  Functions used to assist in other modules within the project.
##

from math import cos, pi, radians

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:      getCartesianAngle
## called by: sensorAnalysis.interpretData()
##            repositionMachine.moveFromObject()
##                              moveToOpponent()
##                              rotateMachine()
## passed:    int round(angle)
## returns:   float customAngle
## calls:     nobody
##
## Returns the standard counter-clockwise Cartesian angle *
## for the given angle.                                   *
## ********************************************************
def getCartesianAngle(angle):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    cartAngle = 0   ##The custom angle to be returned.

    #####################################
    
    if(0 <= angle <= 180):
        cartAngle = 180 - angle
    elif(181 <= angle <= 360):
        cartAngle = 540 - angle
        
    return cartAngle

## ********************************************************
## name:      getCollinearDistance
## called by: sensorAnalysis.interpretData()
##            maneuverAnalysis.calculateObjMovement()
##            repositionMachine.rotateMachine()
## passed:    int x, float SNS_MIN_DISTANCE/SNS_MAX_DISTANCE/
##                         SNS_OPP_DISTANCE/moveObjDistance
## returns:   float newDistance
## calls:     nobody
##
## Returns a custom tracking distance based on the        *
## specified angle.                                       *
## ********************************************************
def getCollinearDistance(newAngle, originAngle, originDistance):

    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    coordAngle = 0.0       ##The angle between originAngle and newAngle values.
    
    newDistance = 0.0      ##The distance at angle newAngle that is in line
                           ##with originDistance, such that there is no slope
                           ##between the two points at each respective angle,
                           ##distance vector location.

    #####################################
    
    coordAngle = abs(originAngle - newAngle)

    newDistance = abs(originDistance/cos(radians(coordAngle)))
        
    return newDistance
