from math import cos, pi, radians, floor, ceil

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
    elif(181 <= angle <= 360):
        cartAngle = 540 - angle
        
    return cartAngle

from adafruit_rplidar import RPLidar

lidar = RPLidar(None, '/dev/ttyUSB0')

sensorDistances = [0.0]*360
count = 0
prevCount = 0
scans = 0
measures = 0

import time
start = time.time()
totalScans = 5
APS = [0]*totalScans

for i, scan in enumerate(lidar.iter_scans()):
 prevCount = count
 measures += len(scan)

 for(_, angle, distance) in scan:
     cartAngle = getCartesianAngle(floor(angle))
     if(sensorDistances[cartAngle] > 0):
        cartAngle = getCartesianAngle(ceil(angle))
        if(sensorDistances[cartAngle] > 0):
            continue

     sensorDistances[cartAngle] = distance * 0.0393
     count += 1
 
 APS[i] = count - prevCount

 if i == totalScans-1 or count == 360:
  scans = i+1
  break

end = time.time()

lidar.stop()
lidar.disconnect()

lastI = -1
blocked = 0

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

print("\n\n", APS)
print("\n\nTime: ", end-start, "\nScans: ", scans, "\nMPS: ", measures/scans, "\nMissing: ", 360-count, "\nBlocked: ", blocked, "\n")
