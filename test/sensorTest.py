from math import cos, pi, radians
import adafruit_rplidar

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
## passed:    int floor(angle)
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
    elif(181 <= angle <= 359):
        cartAngle = 540 - angle
        
    return cartAngle

SENSOR  = adafruit_rplidar.RPLidar(None, '/dev/ttyUSB0')

for scan in SENSOR.iter_scans():

    for i in range (6):
        print(i, ":\n")
        for (_, angle, distance) in scan:
            angle = getCartesianAngle(floor(angle))
            distance *= 0.0393

            if(distance < 20):
                print("Peg: ", angle, distance)
            else:
                print(angle, distance)

