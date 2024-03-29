##  
##  James Garrett
##
##  globals.py
##  Last Updated: May 12, 2022
##
##  Constant declarations and definitions for values needed in other program
##  modules.
##

from math import cos, floor, radians, sin
from controller import Robot	                                
           
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
ROBOT = Robot()
SENSOR  = ROBOT.getDevice('lidar')
WHEELS = [ROBOT.getDevice('wheel1_joint'), ROBOT.getDevice('wheel2_joint'), 
          ROBOT.getDevice('wheel0_joint')]

##Any distance measurements are in inches.
##Any angular measurements are in degrees.

##File directories for indication sounds used throughout the project.
BEGIN_SOUND = "sounds/begin.wav"
FIN_SOUND = "sounds/finished.wav"
BLOKD_SOUND = "sounds/blocked.wav"
STNDBY_SOUND = "sounds/standby.wav"

##Time delay time used before starting and exiting the program.
SLEEP_TIME = 1.33

##The amount of sensor scan rotations to perform for collecting angular distance
##values before analyzing them.
TOTAL_SCANS = 4

##The physical angular location of each wheel on the machine.
WHEEL_LOCATIONS = [210, 330, 90]

##The angular direction the wheels turn when given positive PWM values; that is,
##90 degrees CCW rotation from their location on the machine.
WHEEL_DIRECTIONS = [WHEEL_LOCATIONS[0] - 90, WHEEL_LOCATIONS[1] - 90, 
                    WHEEL_LOCATIONS[2] - 90]

##The following are values used to determine wheel speeds. See documentation for
##further analysis.
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
MAX_SPEED = 20.944 #200 RPM 
TURN_SPEED = 10.472 #100 RPM

##The coefficients for the quadratic equations used to determine the correct PWM
##values for each wheel based on a given RPM speed.
CW_COEFS = [[0.00001, -1.14710, 965.04630],
            [-0.00002, -1.11866, 976.48713],
            [-0.00007, -1.09435, 973.99764]]

CCW_COEFS = [[-0.00003, 1.19735, 1026.95720],
             [0.00006, 1.12871, 1017.63192],
             [0.00007, 1.09813, 1015.55372]]

##The maximum distance value that can be recorded by the lidar sensor according
##to its specifications.
SNS_RANGE = 236.2200

##The front of the machine is considered to be within this angular range and is 
##where the opponent is assumed to be.
FRONT_ANGLE_MIN = 245
FRONT_ANGLE_MAX = 295

##The midpoint of FRONT_ANGLE_MIN and FRONT_ANGLE_MAX, where the machine will 
##look to reposition itself with the opponent.
DES_OPP_ANGLE = round((FRONT_ANGLE_MIN + FRONT_ANGLE_MAX)/2)

##The positions of the opponent in which the machine will turn to face them are
##within these angular ranges.
LEFT_TURN_ANGLE_MIN = 200
LEFT_TURN_ANGLE_MAX = 244
RIGHT_TURN_ANGLE_MIN = 296
RIGHT_TURN_ANGLE_MAX = 340

##If the opponent is unable to move backward, allowing the machine to move
##forward to them, the machine can turn to face them if they are within these
##angular ranges.
STUCK_LEFT_TURN_MIN = 181
STUCK_LEFT_TURN_MAX = 199
STUCK_RIGHT_TURN_MIN = 341
STUCK_RIGHT_TURN_MAX = 359

##The distance from the perimeter of the sensor to the outside of the machine 
##wheels; note: NOT to the machine frame.
MACH_RADIUS = 18.0

##Used for data collection and analysis of angles adjacent to a given angle
##which may not have a usable distance value.
ANGLE_ERR = 7

##The minimum permitted distance for the machine to be from any given object,
##including the opponent.
MIN_DISTANCE = 10.0

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

##The sufficient distance sensor reading.
SNS_SUF_DISTANCE = SUF_DISTANCE + MACH_RADIUS

##Distance used in order to verify that the opponent is beyond the MAX_DISTANCE,
##but still in front of the machine.
OPP_DISTANCE = MAX_DISTANCE + 18.0

##The opponent distance sensor reading.
SNS_OPP_DISTANCE = OPP_DISTANCE + MACH_RADIUS

##Objects cannot be within this angular distance of the machine in the direction
##it is repositioning if also closer than the distance being traveled while
##repositioning.
PATH_ZONE = 90


