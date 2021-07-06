from math import cos, floor, pi, radians
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
sensorDistances = [0]*360
count = 0
prevCount = 0

for scan in SENSOR.iter_scans():
    prevCount = count

    for (_, angle, distance) in scan:
        angle = getCartesianAngle(floor(angle))

        if(sensorDistances[angle] > 0):
            continue

        sensorDistances[angle] = distance * 0.0393
        count += 1
       
    if(count == prevCount):
        break

print("\n", count, "\n")
for i in range(360):
    if(0.0 < sensorDistances[i] < 20.0):
        print (i, sensorDistances[i])

