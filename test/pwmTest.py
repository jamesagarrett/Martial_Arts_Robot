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
    val = 0     ##The corresponding PWM value for the given wheel RPM.
  
    CCW_COEFS = [[-0.0002, 1.3091, 1051.4177],
                 [-0.0001, 1.2512, 1028.5241],
                 [-0.0000, 1.3511, 1027.8794]]

    CW_COEFS = [[0.0003, -1.3877, 958.0234],
                [0.0000, -1.2198, 968.0393],
                [0.0001, -1.3325, 963.6987]]

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

wheelSpeeds[1] = (speedVars[0] - wheelSpeeds[0]*SPEED_CONSTS[1]) \
					/ SPEED_CONSTS[3]

wheelSpeeds[2] = (-wheelSpeeds[1] - wheelSpeeds[0] + rotationCoeff)

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
time.sleep(0.25)

for x in range (3):
    if (wantRPMs[x] < 5):
        realRPMs[x] = 0
    else:
        realRPMs[x] = getRPMs[pins[x]]

print("Speeds: ", wheelSpeeds)
print("PWMs: ", wheelPWMs)
print("RPMs (Wanted/Actual): %.2f/%.2f, %.2f/%.2f, %.2f/%.2f" % (wantRPMs[0], realRPMs[0], wantRPMs[1], realRPMs[1], wantRPMs[2], realRPMs[2]))

try:
	while(True):
		continue
except KeyboardInterrupt:
    pwm.set_pwm(PWM_PORTS[0], 0, 1000)
    pwm.set_pwm(PWM_PORTS[1], 0, 1000)
    pwm.set_pwm(PWM_PORTS[2], 0, 1000)
