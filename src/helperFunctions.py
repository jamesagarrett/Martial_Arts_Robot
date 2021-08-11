##  
##  James Garrett
##
##  helperFunctions.py
##  Last Updated: August 9, 2021
##
##  Functions used to assist with other modules within the project.
##

from math import cos, pi, radians

##THESE VALUES ARE NOT TO BE CHANGED!
from globals import PATH_ZONE

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##	FUNCTION DEFINITION
##
#########################################

## ********************************************************
## name:      getCartesianAngle
## called by: lidarAnalysis.interpretData()
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
## called by: lidarAnalysis.interpretData()
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

## ********************************************************
## name:      getPathAngles
## called by: maneuverAnalysis.calculateObjMovement(),
##            repositionMachine.moveToOpponent()
## passed:    int maneuverAngle/DES_OPP_ANGLE
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

