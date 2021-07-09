from math import cos, pi, radians, floor
import adafruit_rplidar

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#
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

SENSOR  = adafruit_rplidar.RPLidar('/dev/ttyUSB0')
SENSOR.reset()
sensorDistances = [0]*360
count = 0
prevCount = 0
blocked = 0

for scan in SENSOR.iter_scans():
    prevCount = count

    for (_, angle, distance) in scan:
        angle = getCartesianAngle(round(angle))

        if(sensorDistances[angle] > 0):
            continue

        sensorDistances[angle] = distance * 0.0393
        count += 1
        if(count % 10 == 0):
            print(count)
       
    if(count == prevCount):
        break

SENSOR.stop()
SENSOR.stop_motor()
SENSOR.disconnect()

lastI = -1

for i in range(360):
    if(sensorDistances[i] == 0.0):
        if(i != lastI + 1):
            print("\n")
        
        print (i, "NONE")
        lastI = i

    if(0.0 < sensorDistances[i] < 20.0):
        if(i != lastI + 1):
            print("\n")

        print (i, sensorDistances[i])
        blocked += 1
        lastI = i

print("\n\nMissing: ", 360-count, "\nBlocked: ", blocked, "\n")
