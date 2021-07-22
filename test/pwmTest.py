#Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
from math import sin, cos, radians

# Import the PCA9685 module.
import PCA9685

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685.PCA9685()

pwm.set_pwm_freq(160)
maxSpeed = 325
CCW_MIN = 1025
CW_MIN = 965
speedBoost = 0

#x = float(input("Speed: "))
#y = float(input("Speed: "))
#z = float(input("Speed: "))

#while(x != 0 and y != 0 and z != 0):
#	if(x > 0):
#		speed1 = CCW_MIN + speedBoost + maxSpeed * x
#	else:
#		speed1 = CW_MIN - speedBoost + maxSpeed * x
	
#	if(y > 0):
#		speed2 = CCW_MIN + maxSpeed * y
#	else:
#		speed2 = CW_MIN + maxSpeed * y
	
#	if(z > 0):
#		speed3 = CCW_MIN + maxSpeed * z
#	else:
#		speed3 = CW_MIN + maxSpeed * z

#	print(speed1, speed2, speed3)
#	pwm.set_pwm(7, 0, math.floor(speed1))
#	pwm.set_pwm(5, 0, math.floor(speed2))
#	pwm.set_pwm(6, 0, math.floor(speed3))
	
#	x = float(input("Speed: "))
#	y = float(input("Speed: "))
#	z = float(input("Speed: "))

#pwm.set_pwm(7, 0, 995)
#pwm.set_pwm(5, 0, 995)
#pwm.set_pwm(6, 0, 995)

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
        wheelPWMs[x] = 995
    elif(wheelSpeeds[x] > 0):
        wheelPWMs[x] = round(CCW_MIN + maxSpeed * wheelSpeeds[x])
    else:
        wheelPWMs[x] = round(CW_MIN + maxSpeed * wheelSpeeds[x])
 
print("Speeds: ", wheelSpeeds)
print("PWMs: ", wheelPWMs)

try:
	while(True):
		pwm.set_pwm(PWM_PORTS[0], 0, wheelPWMs[0])
		pwm.set_pwm(PWM_PORTS[1], 0, wheelPWMs[1])
		pwm.set_pwm(PWM_PORTS[2], 0, wheelPWMs[2])
except KeyboardInterrupt:
    pwm.set_pwm(PWM_PORTS[0], 0, 995)
    pwm.set_pwm(PWM_PORTS[1], 0, 995)
    pwm.set_pwm(PWM_PORTS[2], 0, 995)
