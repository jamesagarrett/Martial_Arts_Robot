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

x = float(input("Speed: "))
y = float(input("Speed: "))
z = float(input("Speed: "))

while(x != 0 and y != 0 and z != 0):
	if(x < -1):
		speed = 1025 + (150/(1/x))
	else:
		speed = 965 + (150/(1/x))
	
	if(y < -1):
		speed2 = 1025 + (150/(1/y))
	else:
		speed2 = 965 + (150/(1/y))
	
	if(z < -1):
		speed3 = 1025 + (150/(1/z))
	else:
		speed3 = 965 + (150/(1/z))

	print(speed, speed2, speed3)
	pwm.set_pwm(7, 0, math.floor(speed))
	pwm.set_pwm(6, 0, math.floor(speed2))
	pwm.set_pwm(5, 0, math.floor(speed3))
	
	time.sleep(5)
	x = float(input("Speed: "))
	y = float(input("Speed: "))
	z = float(input("Speed: "))

#to stop robot at the end
pwm.set_pwm(7, 0, 1000)
pwm.set_pwm(6, 0, 1000)
pwm.set_pwm(5, 0, 1000)

