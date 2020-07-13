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

x = -0.47956
while(x != 0):
    if(-1 <= x <= 1):
        if(x < 0):
            speed = 1025 + (150/(1/x))
        else:
            speed = 965 + (150/(1/x))
        print(speed)
        pwm.set_pwm(7, 0, math.floor(speed))
        pwm.set_pwm(0, 0, math.floor(speed))
        pwm.set_pwm(15, 0, math.floor(speed))
    else:
        print("Invalid")
    x = float(input("Speed: "))

#to stop robot at the end
pwm.set_pwm(7, 0, 1000)
pwm.set_pwm(0, 0, 1000)
pwm.set_pwm(15, 0, 1000)

