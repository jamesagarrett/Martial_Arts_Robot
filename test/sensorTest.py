from math import cos, pi, radians, floor

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

from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0')
sensorDistances = [0.0]*360
count = 0
prevCount = 0

for i, scan in enumerate(lidar.iter_scans()):
 print('%d: Got %d measures' % (i, len(scan)))
 for(_, angle, distance) in scan:
     angle = getCartesianAngle(round(angle))

     if(sensorDistances[angle] > 0):
        continue

     sensorDistances[angle] = distance * 0.0393
     count += 1

 if i == 40 or count == 360 or prevCount == count:
  break

lidar.stop()
lidar.stop_motor()
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

print("\n\nMissing: ", 360-count, "\nBlocked: ", blocked, "\n")