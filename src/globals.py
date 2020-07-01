##  
##  James Garrett
##
##  Martial_Arts_Robot 
##  Last Updated: January 19, 2020
##
##  globals.py
##  Last Updated: August 12, 2019
##
##  Constant declarations and definitions for values needed in other program
##  modules.
##

##Slamtec RPLIDAR A1 - 360 Laser Range Scanner Library;
##Created by MIT Skoltech, edited by and retrieved through Adafruit;
##See adafruit_rplidar.py for copyright license. 
#import adafruit_rplidar

##Adafruit 16-Channel 12-bit PWM/Servo Driver Library;
##Created by and retrieved through Adafruit;
##See PCA9685.py for copyright license. 
#import PCA9685	                                
           
##Allows for calculations, conversions, and rounding.
from math import ceil, cos, floor, radians, sin 

#--------------------------------------  --------------------------------------#
#--------------------------------------  --------------------------------------#

#########################################
##
##  CONSTANT DECLARATION
##
#########################################

##VALUES DEFINED BELOW SHOULD ONLY BE ALTERED UNDER THE FOLLOWING CONDITIONS:
##
##      1.  There are physical design changes to the machine, such as wheel 
##          positioning.
##      2.  Troubleshooting and testing were used to determine other default 
##          values are more practical.

##Global objects for the LIDAR sensor and pwm driver breakout board.
WHEELS = 1#PCA9685.PCA9685()
SENSOR  = 1#adafruit_rplidar.RPLidar(None, '/dev/ttyUSB0')

##Any distance measurements are assumed to be in inches.
##Any angular measurements are assumed to be in degrees.

##The pwm frequency used for driving the wheels.
PWM_FREQ = 160

##The pin location of each wheel on the pwm breakout board.
PWM_PORTS = [5, 6, 7]

##The physical angular location of each wheel on the machine.
WHEEL_LOCATIONS = [210, 330, 90]

##The angular direction the wheels turn when given positive output values;
##that is, 90 deg (pi/2 rad) ccw rotation from their location on the machine.
WHEEL_DIRECTIONS = [WHEEL_LOCATIONS[0] - 90, WHEEL_LOCATIONS[1] - 90, 
                    WHEEL_LOCATIONS[2] - 90]

##The following are values used to determine wheel speeds. See documentation 
##for further analysis. Due to how these equations are used, certain 
##precautions must be taken when determining WHEEL_LOCATIONS values. This is 
##also further explained in the documentation.
SPEED_CONSTS = [0]*4

SPEED_CONSTS[0] = cos(radians(WHEEL_DIRECTIONS[0])) \
                - cos(radians(WHEEL_DIRECTIONS[2]))

SPEED_CONSTS[1] = sin(radians(WHEEL_DIRECTIONS[0])) \
                  - sin(radians(WHEEL_DIRECTIONS[2]))

SPEED_CONSTS[2] = cos(radians(WHEEL_DIRECTIONS[1])) \
                  - cos(radians(WHEEL_DIRECTIONS[2]))

SPEED_CONSTS[3] = sin(radians(WHEEL_DIRECTIONS[1])) \
                  - sin(radians(WHEEL_DIRECTIONS[2]))

##The tick value in which the signal to the wheels transitions low to high.
START_TICK = 0

##The tick values in which the signal to the wheels transitioning from high to 
##low will result in no movement. Any value above MIN_CCW_SPEED or below 
##MIN_CW_SPEED will result in movement by the wheels. STOP_SPEED is the mean of
##these values.
MIN_CCW_SPEED = 1025
MIN_CW_SPEED = 965
STOP_SPEED = floor((MIN_CCW_SPEED + MIN_CW_SPEED)/2)

##The desired value above MIN_CCW_SPEED or below MIN_CW_SPEED to be the 
##maximum speed for the wheels.
MAX_SPEED = 200
MAX_CCW_SPEED = MIN_CCW_SPEED + MAX_SPEED
MAX_CW_SPEED = MIN_CW_SPEED - MAX_SPEED

##The front of the machine is considered to be within this angular range and is 
##where the opponent is assumed to be.
FRONT_ANGLE_MIN = 240
FRONT_ANGLE_MAX = 299

##The midpoint of FRONT_ANGLE_MIN and FRONT_ANGLE_MAX, where the machine will 
##look to reposition itself with the opponent.
DES_OPP_ANGLE = ceil((FRONT_ANGLE_MIN + FRONT_ANGLE_MAX)/2)

##The angles in which two sides meet - the vertices - used for calculations 
##before repositioning toward an object.
VRTX_ANGLES = [30, 90, 150, 210, 270, 330]

##The positions of the opponent in which the machine will turn to face them, are
##considered to be within these angular ranges.
LEFT_TURN_ANGLE_MIN = 180
LEFT_TURN_ANGLE_MAX = 239
RIGHT_TURN_ANGLE_MIN = 300
RIGHT_TURN_ANGLE_MAX = 359

##The distance from the perimeter of the sensor to the outside of the machine 
##wheels; note: NOT to the machine frame.
MACH_RADIUS = 20.0

##The number of angles permitted beyond any given angle that all have an 
#associated distance that is unknown - value of 0 - or is less than or equal to 
#the MACH_RADIUS.
ANGLE_ERR = 3

##The length of a single side of the machine frame; used for calculations 
##before repositioning toward an object.
SIDE_LENGTH = 16.0

##The minimum permitted distance for the machine to be from any given object,
##including the opponent.
MIN_DISTANCE = 9.0

##The minimum permitted distance sensor reading.
SNS_MIN_DISTANCE = MIN_DISTANCE + MACH_RADIUS

##The maximum permitted distance for the machine to be from the opponent it is 
##tracking.
MAX_DISTANCE = 60.0

##The maximum permitted distance sensor reading.
SNS_MAX_DISTANCE = MAX_DISTANCE + MACH_RADIUS

##Distance used in order to verify that the opponent is beyond the MAX_DISTANCE,
##but still in front of the machine.
OPP_DISTANCE = MAX_DISTANCE + 6.0

##The opponent distance sensor reading.
SNS_OPP_DISTANCE = OPP_DISTANCE + MACH_RADIUS

##Objects too close to the machine must be this angular distance from each 
##other for the machine to traverse between them.
OBJ_ANGLE_MIN = 120

##The (optimal) preferred distance for the machine to be from any given object, 
##including the opponent.
OPT_DISTANCE = (MAX_DISTANCE + MIN_DISTANCE)/3

##The optimal distance sensor reading.
SNS_OPT_DISTANCE = OPT_DISTANCE + MACH_RADIUS

##The (sufficient) distance used if the OPT_DISTANCE value cannot be reached 
##when repositioning.
SUF_DISTANCE = (OPT_DISTANCE + MIN_DISTANCE)/2


