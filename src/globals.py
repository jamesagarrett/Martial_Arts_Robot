##  
##  James Garrett
##
##  globals.py
##  Last Updated: August 8, 2021
##
##  Constant declarations and definitions for values needed in other program
##  modules.
##

from math import cos, floor, radians, sin 

##Slamtec RPLIDAR A1 - 360 Laser Range Scanner Library;
##Created by MIT Skoltech, edited by and retrieved through Adafruit;
##See adafruit_rplidar.py for copyright license. 
import adafruit_rplidar

##Adafruit 16-Channel 12-bit PWM/Servo Driver Library;
##Created by and retrieved through Adafruit;
##See PCA9685.py for copyright license. 
import PCA9685	                                
           
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
##      2.  Testing is performed and determines other default values are more
##          practical.

##Global objects for the LIDAR sensor and PWM driver breakout board.
SENSOR  = adafruit_rplidar.RPLidar(None, '/dev/ttyUSB0')
WHEELS = PCA9685.PCA9685()

##Any distance measurements are in inches.
##Any angular measurements are in degrees.

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
SPEED_CONSTS = [
cos(radians(WHEEL_DIRECTIONS[0])) - cos(radians(WHEEL_DIRECTIONS[2])),
sin(radians(WHEEL_DIRECTIONS[0])) - sin(radians(WHEEL_DIRECTIONS[2])),
cos(radians(WHEEL_DIRECTIONS[1])) - cos(radians(WHEEL_DIRECTIONS[2])),
sin(radians(WHEEL_DIRECTIONS[1])) - sin(radians(WHEEL_DIRECTIONS[2]))]

##The PWM frequency used for driving the wheels.
PWM_FREQ = 160

##The pin location of each wheel on the PWM breakout board.
PWM_PORTS = [7, 5, 6]

##The tick value in which the signal to the wheels transitions low to high.
START_TICK = 0

##The tick values in which the signal to any wheel transitioning from high to 
##low will result in no movement. 
STOP_TICK = 1000

##Wheel RPM speeds that will be used for determining PWM values for
##repositioning.
MAX_SPEED = 400 
TURN_SPEED = 90

##The coefficients for the quadratic equations used to determine the correct PWM
##values for each wheel based on a given RPM speed.
CCW_COEFS = [[-0.00003, 1.19735, 1026.95720],
             [0.00006, 1.12871, 1017.63192],
             [0.00007, 1.09813, 1015.55372]]

CW_COEFS = [[0.00001, -1.14710, 965.04630],
            [-0.00002, -1.11866, 976.48713],
            [-0.00007, -1.09435, 973.99764]]

##The amount of sensor scan rotations to perform for collecting angular distance
##values before analyzing them.
TOTAL_SCANS = 3

##The front of the machine is considered to be within this angular range and is 
##where the opponent is assumed to be.
FRONT_ANGLE_MIN = 250
FRONT_ANGLE_MAX = 289

##The midpoint of FRONT_ANGLE_MIN and FRONT_ANGLE_MAX, where the machine will 
##look to reposition itself with the opponent.
DES_OPP_ANGLE = round((FRONT_ANGLE_MIN + FRONT_ANGLE_MAX)/2)

##The positions of the opponent in which the machine will turn to face them, are
##considered to be within these angular ranges.
LEFT_TURN_ANGLE_MIN = 180
LEFT_TURN_ANGLE_MAX = 249
RIGHT_TURN_ANGLE_MIN = 290
RIGHT_TURN_ANGLE_MAX = 359

##The distance from the perimeter of the sensor to the outside of the machine 
##wheels; note: NOT to the machine frame.
MACH_RADIUS = 19.0

##The number of angles permitted beyond any given angle that all have an 
#associated distance that is unknown - value of 0 - or is less than or equal to 
#the MACH_RADIUS.
ANGLE_ERR = 4

##The minimum permitted distance for the machine to be from any given object,
##including the opponent.
MIN_DISTANCE = 7.0

##The minimum permitted distance sensor reading.
SNS_MIN_DISTANCE = MIN_DISTANCE + MACH_RADIUS

##The maximum permitted distance for the machine to be from the opponent it is 
##tracking.
MAX_DISTANCE = 36.0

##The maximum permitted distance sensor reading.
SNS_MAX_DISTANCE = MAX_DISTANCE + MACH_RADIUS

##The optimal and preferred distance for the machine to be from any given 
##object, including the opponent.
OPT_DISTANCE = 24.0

##The optimal distance sensor reading.
SNS_OPT_DISTANCE = OPT_DISTANCE + MACH_RADIUS

##The sufficient distance used if the OPT_DISTANCE value cannot be reached when
##repositioning.
SUF_DISTANCE = 14.0

##Distance used in order to verify that the opponent is beyond the MAX_DISTANCE,
##but still in front of the machine.
OPP_DISTANCE = MAX_DISTANCE + 18.0

##The opponent distance sensor reading.
SNS_OPP_DISTANCE = OPP_DISTANCE + MACH_RADIUS

##Objects cannot be within this angular distance of the machine in the direction
##it is repositioning if also closer than the distance being traveled while
##repositioning.
PATH_ZONE = 90


