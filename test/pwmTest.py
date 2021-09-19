#Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
from math import sin, cos, radians
import PCA9685
import RPi.GPIO as GPIO

pwm = PCA9685.PCA9685()
pwm.set_pwm_freq(160)
PWM_PORTS = [7, 5, 6]

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
GPIO.setup(4, GPIO.IN)
GPIO.setup(10, GPIO.IN)
GPIO.setwarnings(False)

def calculatePWM(wheel, rpm, spinCCW):
    val = 0     
    
    CCW_COEFS = [[-0.00003, 1.19735, 1026.95720],
                 [0.00006, 1.12871, 1017.63192],
                 [0.00007, 1.09813, 1015.55372]]

    CW_COEFS = [[0.00001, -1.14710, 965.04630],
                [-0.00002, -1.11866, 976.48713],
                [-0.00007, -1.09435, 973.99764]]

    if(spinCCW):
        val = CCW_COEFS[wheel][0] * rpm**2 \
            + CCW_COEFS[wheel][1] * rpm \
            + CCW_COEFS[wheel][2]
    else:
        val = CW_COEFS[wheel][0] * rpm**2 \
            + CW_COEFS[wheel][1] * rpm \
            + CW_COEFS[wheel][2]
    
    return round(val)

def getRPM(pin):
    try:
        revolvs = 1
        average = 0

        for _ in range (revolvs):
            while(GPIO.input(pin) == True):
                continue

            while(GPIO.input(pin) == False):
                start = time.time()
            while(GPIO.input(pin) == True):
                end = time.time()

            revolution = (1/(end-start))*60
            average += revolution

        average /= revolvs
        print(pin, average)
        return average

    except KeyboardInterrupt:
        pwm.set_pwm(PWM_PORTS[0], 0, 1000)
        pwm.set_pwm(PWM_PORTS[1], 0, 1000)
        pwm.set_pwm(PWM_PORTS[2], 0, 1000)
        quit()

maxSpeed = 400
wheelSpeeds = [0]*3             
speedVars = [0]*2
wheelPWMs = [0]*3
wantRPMs = [0]*3
realRPMs = [0]*3
errRPMs = [0]*3
pins = [4, 10, 21]

WHEEL_LOCATIONS = [210, 330, 90]

WHEEL_DIRECTIONS = [WHEEL_LOCATIONS[0] - 90, WHEEL_LOCATIONS[1] - 90, 
                    WHEEL_LOCATIONS[2] - 90]

SPEED_CONSTS = [
cos(radians(WHEEL_DIRECTIONS[0])) - cos(radians(WHEEL_DIRECTIONS[2])),
sin(radians(WHEEL_DIRECTIONS[0])) - sin(radians(WHEEL_DIRECTIONS[2])),
cos(radians(WHEEL_DIRECTIONS[1])) - cos(radians(WHEEL_DIRECTIONS[2])),
sin(radians(WHEEL_DIRECTIONS[1])) - sin(radians(WHEEL_DIRECTIONS[2]))]

angle = int(input("Angle: "))

speedVars[0] = 1*sin(radians(angle)) \
				- sin(radians(WHEEL_DIRECTIONS[2]))

speedVars[1] = 1*cos(radians(angle)) \
				- cos(radians(WHEEL_DIRECTIONS[2]))

wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
					- speedVars[0]*SPEED_CONSTS[2]) \
					/ (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
					- SPEED_CONSTS[1]*SPEED_CONSTS[2]) 

wheelSpeeds[0] = round(wheelSpeeds[0], 5)

wheelSpeeds[1] = round((speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) / SPEED_CONSTS[3], 5)

wheelSpeeds[2] = round((-wheelSpeeds[1] - wheelSpeeds[0]), 5)
print("Speeds: ", wheelSpeeds)

for x in range (3):
    if(wheelSpeeds[x] == 0):
        wheelPWMs[x] = 1000
        wantRPMs[x] = 0
    elif(wheelSpeeds[x] > 0):
        wantRPMs[x] = wheelSpeeds[x]*maxSpeed
        wheelPWMs[x] = calculatePWM(x, wantRPMs[x], True)
    else:
        wantRPMs[x] = abs(wheelSpeeds[x]*maxSpeed)
        wheelPWMs[x] = calculatePWM(x, wantRPMs[x], False)

pwm.set_pwm(PWM_PORTS[0], 0, wheelPWMs[0])
pwm.set_pwm(PWM_PORTS[1], 0, wheelPWMs[1])
pwm.set_pwm(PWM_PORTS[2], 0, wheelPWMs[2])
time.sleep(3)
print("Speeds: ", wheelSpeeds)
print("PWMs: ", wheelPWMs)

pwm.set_pwm(PWM_PORTS[0], 0, 1000)
pwm.set_pwm(PWM_PORTS[1], 0, 1000)
pwm.set_pwm(PWM_PORTS[2], 0, 1000)

#for x in range (3):
#    if (wantRPMs[x] < 1):
#        realRPMs[x] = 0
#    else:
#        realRPMs[x] = getRPM(pins[x])

#for x in range (3):
#	if(wantRPMs[x] != 0):
#		errRPMs[x] = ((realRPMs[x]-wantRPMs[x])/wantRPMs[x])*100 
#	else:
#		errRPMs[x] = realRPMs[x]

#print("RPMs (Actual/Expected): %.2f/%.2f (%.2f%%), %.2f/%.2f (%.2f%%), %.2f/%.2f (%.2f%%)" % (realRPMs[0], wantRPMs[0], errRPMs[0], realRPMs[1], wantRPMs[1], errRPMs[1], realRPMs[2], wantRPMs[2], errRPMs[2]))


