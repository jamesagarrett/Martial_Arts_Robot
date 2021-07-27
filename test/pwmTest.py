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

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
GPIO.setup(4, GPIO.IN)
GPIO.setup(10, GPIO.IN)
GPIO.setwarnings(False)

def calculatePWM(wheel, rpm, spinCCW):
    val = 0     
    
    CCW_COEFS = [[-0.0002, 1.3091, 1051.4177],
                 [-0.0001, 1.2512, 1028.5241],
                 [-0.0000, 1.3511, 1027.8794]]

    CW_COEFS = [[0.0003, -1.3877, 958.0234],
                [0.0000, -1.2198, 968.0393],
                [0.0001, -1.3325, 963.6987]]

    #CCW_COEFS = [[-0.00025, 1.36191, 1044.37192],
    #             [0.00004, 1.26774, 1028.70953],
    #             [-0.00012, 1.48888, 1020.06205]]

    #CW_COEFS = [[0.00029, -1.40433, 961.66443],
    #            [0.00002, -1.25530, 968.15763],
    #            [0.00028, -1.46364, 979.17510]]

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
    revolvs = 10
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
    return average

maxSpeed = 200
wheelSpeeds = [0]*3             
speedVars = [0]*2
rotationCoeff = 0
wheelPWMs = [0]*3
wantRPMs = [0]*3
realRPMs = [0]*3
errRPMs = [0]*3
PWM_PORTS = [7, 5, 6]
pins = [21, 4, 10]

WHEEL_LOCATIONS = [210, 330, 90]

WHEEL_DIRECTIONS = [WHEEL_LOCATIONS[0] - 90, WHEEL_LOCATIONS[1] - 90, 
                    WHEEL_LOCATIONS[2] - 90]

SPEED_CONSTS = [
cos(radians(WHEEL_DIRECTIONS[0])) - cos(radians(WHEEL_DIRECTIONS[2])),
sin(radians(WHEEL_DIRECTIONS[0])) - sin(radians(WHEEL_DIRECTIONS[2])),
cos(radians(WHEEL_DIRECTIONS[1])) - cos(radians(WHEEL_DIRECTIONS[2])),
sin(radians(WHEEL_DIRECTIONS[1])) - sin(radians(WHEEL_DIRECTIONS[2]))]

angle = int(input("Angle: "))

speedVars[0] = sin(radians(angle)) \
				- rotationCoeff*sin(radians(WHEEL_DIRECTIONS[2]))

speedVars[1] = cos(radians(angle)) \
				- rotationCoeff*cos(radians(WHEEL_DIRECTIONS[2]))

wheelSpeeds[0] = (speedVars[1]*SPEED_CONSTS[3] \
					- speedVars[0]*SPEED_CONSTS[2]) \
					/ (SPEED_CONSTS[0]*SPEED_CONSTS[3] \
					- SPEED_CONSTS[1]*SPEED_CONSTS[2]) 
wheelSpeeds[0] = round(wheelSpeeds[0], 5)

wheelSpeeds[1] = round((speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) / SPEED_CONSTS[3], 5)

wheelSpeeds[2] = round((-wheelSpeeds[1] - wheelSpeeds[0] + rotationCoeff), 5)

for x in range (3):
    if(wheelSpeeds[x] == 0):
        wheelPWMs[x] = 1000
        wantRPMs[x] = 0
    elif(wheelSpeeds[x] > 0):
        wantRPMs[x] = wheelSpeeds[x]*maxSpeed
        #wheelPWMs[x] = round(1045 + wantRPMs[x])
        wheelPWMs[x] = calculatePWM(x, wantRPMs[x], True)
    else:
        wantRPMs[x] = abs(wheelSpeeds[x]*maxSpeed)
        #wheelPWMs[x] = round(955 - wantRPMs[x])
        wheelPWMs[x] = calculatePWM(x, wantRPMs[x], False)

pwm.set_pwm(PWM_PORTS[0], 0, wheelPWMs[0])
pwm.set_pwm(PWM_PORTS[1], 0, wheelPWMs[1])
pwm.set_pwm(PWM_PORTS[2], 0, wheelPWMs[2])
time.sleep(0.1)

for x in range (3):
    if (wantRPMs[x] < 1):
        realRPMs[x] = 0
    else:
        realRPMs[x] = getRPM(pins[x])

for x in range (3):
	if(wantRPMs[x] != 0):
		errRPMs[x] = ((realRPMs[x]-wantRPMs[x])/wantRPMs[x])*100 
	else:
		errRPMs[x] = realRPMs[x]

print("Speeds: ", wheelSpeeds)
print("PWMs: ", wheelPWMs)
print("RPMs (Actual/Expected): %.2f/%.2f (%.2f%%), %.2f/%.2f (%.2f%%), %.2f/%.2f (%.2f%%)" % (realRPMs[0], wantRPMs[0], errRPMs[0], realRPMs[1], wantRPMs[1], errRPMs[1], realRPMs[2], wantRPMs[2], errRPMs[2]))

#try:
#	while(True):
#		continue
#except KeyboardInterrupt:
pwm.set_pwm(PWM_PORTS[0], 0, 1000)
pwm.set_pwm(PWM_PORTS[1], 0, 1000)
pwm.set_pwm(PWM_PORTS[2], 0, 1000)
