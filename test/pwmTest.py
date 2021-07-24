#Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
from math import sin, cos, radians

# Import the PCA9685 module.
import PCA9685

def calculatePWM(wheel, rpm, spinCCW):
    #####################################
    ##
    ##  VARIABLE DECLARATION
    ##
    #####################################

    val = 0     ##The corresponding PWM value for the given wheel RPM.
    
    #####################################

    if(spinCCW):
        val = CCW_COEFS[wheel][0] * rpm**2 \
            + CCW_COEFS[wheel][1] * rpm \
            + CCW_COEFS[wheel][2]
    else:
        val = CW_COEFS[wheel][0] * rpm**2 \
            + CW_COEFS[wheel][1] * rpm \
            + CW_COEFS[wheel][2]
    
    return round(val)

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685.PCA9685()
pwm.set_pwm_freq(160)

maxSpeed = 200
wheelSpeeds = [0]*3             
speedVars = [0]*2
rotationCoeff = 0
wheelPWMs = [0]*3

PWM_PORTS = [7, 5, 6]

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
    elif(wheelSpeeds[x] > 0):
        wheelPWMs[x] = calculatePWM(x, wheelSpeeds[x]*maxSpeed, True)
    else:
        wheelPWMs[x] = calculatePWM(x, abs(wheelSpeeds[x]*maxSpeed), False)
 
print("Speeds: ", wheelSpeeds)
print("RPMs: ", abs(wheelSpeeds[0]*maxSpeed), abs(wheelSpeeds[1]*maxSpeed), abs(wheelSpeeds[2]*maxSpeed))
print("PWMs: ", wheelPWMs)

try:
	while(True):
		pwm.set_pwm(PWM_PORTS[0], 0, wheelPWMs[0])
		pwm.set_pwm(PWM_PORTS[1], 0, wheelPWMs[1])
		pwm.set_pwm(PWM_PORTS[2], 0, wheelPWMs[2])
except KeyboardInterrupt:
    pwm.set_pwm(PWM_PORTS[0], 0, 1000)
    pwm.set_pwm(PWM_PORTS[1], 0, 1000)
    pwm.set_pwm(PWM_PORTS[2], 0, 1000)
