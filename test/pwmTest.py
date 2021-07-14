#Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import math

# Import the PCA9685 module.
import PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:?_
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

pwm.set_pwm_freq(160)
maxSpeed = 200
CCW_MIN = 1025
CW_MIN = 965

x = float(input("Speed: "))
y = float(input("Speed: "))
z = float(input("Speed: "))

while(x != 0 and y != 0 and z != 0):
	if(x > 0):
		speed1 = CCW_MIN + 20 + maxSpeed * x
	else:
		speed1 = CW_MIN - 20 + maxSpeed * x
	
	if(y > 0):
		speed2 = CCW_MIN + maxSpeed * y
	else:
		speed2 = CW_MIN + maxSpeed * y
	
	if(z > 0):
		speed3 = CCW_MIN + maxSpeed * z
	else:
		speed3 = CW_MIN + maxSpeed * z

	print(speed1, speed2, speed3)
	pwm.set_pwm(7, 0, math.floor(speed1))
	pwm.set_pwm(6, 0, math.floor(speed2))
	pwm.set_pwm(5, 0, math.floor(speed3))
	
	x = float(input("Speed: "))
	y = float(input("Speed: "))
	z = float(input("Speed: "))

#to stop robot at the end
pwm.set_pwm(7, 0, 1000)
pwm.set_pwm(6, 0, 1000)
pwm.set_pwm(5, 0, 1000)

