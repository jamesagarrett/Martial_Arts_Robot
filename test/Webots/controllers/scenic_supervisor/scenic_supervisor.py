
from controller import Supervisor
from random import randint, uniform
from math import atan, ceil, degrees, floor, cos, sin, radians as rad


#[-0.7165161866979263, -1.2410424398063153, 0.0] case didnt work
MIN_DISTANCE = 0.7112
MAX_DISTANCE = 1.3716
OPP_DISTANCE = 1.8288
MACH_RADIUS = 0.4572

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

LEFT_MDPT = round((LEFT_TURN_ANGLE_MIN + LEFT_TURN_ANGLE_MAX)/2)
RIGHT_MDPT = round((RIGHT_TURN_ANGLE_MIN + RIGHT_TURN_ANGLE_MAX)/2)

def getPosition():
    """
        1) randomly pick angle from 180 to 359
        2) 3 if statements for diff regions
        3) case 1: dark green/ red region
            a) further split into < 270 and > 270
            b) calculate length of hyp of light and dark green regions using angle (abs value diff
                of angle and 270)
                and midpoint line
                randomly select distance between machine radius to min distance 
                or within dark green region
            c) calculate the x and y coordinates using trig laws
        4) case 2: left side light blue region
            a) further split into < 210 and > 210
            b) calculate the hyp formed from angle measure using the midpoint line
                and angle (abs value diff of angle and 210) 
                randomly pick distance from machine radius to hyp calculated
           c) calculate the x and y coordinates using trig laws
        5) case 3: right side light blue region
            a) further split into < 330 and > 330
            b) calculate the hyp formed from angle measure using the midpoint line
                and angle (abs value diff of angle and 330) 
                randomly pick distance from machine radius to hyp calculated
            c) calculate the x and y coordinates using trig laws
            
    """
    angle = randint(LEFT_TURN_ANGLE_MIN,RIGHT_TURN_ANGLE_MAX)
    #light blue left region
    if angle < FRONT_ANGLE_MIN:
        helperAngle = abs(angle - LEFT_MDPT)
        hyp = MAX_DISTANCE / cos(rad(helperAngle))
        pos = uniform(MACH_RADIUS + 0.0001, hyp)
        x = cos(rad(angle - 180)) * pos
        y = sin(rad(angle - 180)) * pos
        return [x*-1,y*-1]
    #dark green region
    elif angle < RIGHT_TURN_ANGLE_MIN:
        helperAngle = abs(angle - DES_OPP_ANGLE)
        lightHyp = MAX_DISTANCE / cos(rad(helperAngle))
        darkHyp = OPP_DISTANCE / cos(rad(helperAngle))
        pos = uniform(MACH_RADIUS + 0.0001, darkHyp)
        while pos >= MIN_DISTANCE and pos <= lightHyp:
            pos = uniform(MACH_RADIUS + 0.0001, darkHyp)
        y = cos(rad(abs(DES_OPP_ANGLE - angle))) * pos
        x = sin(rad(abs(DES_OPP_ANGLE - angle))) * pos
        if angle < DES_OPP_ANGLE:
            x *= -1
        return [x,y*-1]
    else:
        helperAngle = abs(angle - RIGHT_MDPT)
        hyp = MAX_DISTANCE / cos(rad(helperAngle))
        pos = uniform(MACH_RADIUS + 0.0001, hyp)
        x = cos(rad(360 - angle)) * pos
        y = sin(rad(360 - angle)) * pos
        return [x,y*-1]
    
    
    


def main():
    supervisor = Supervisor()
    
    # set positions of the robot and pedestrian
    human_node = supervisor.getFromDef("HUMAN")
    trans_field = human_node.getField("translation")
    print(trans_field.getSFVec3f())
    pos = getPosition()
    trans_field.setSFVec3f([pos[0],pos[1],0])
    print(trans_field.getSFVec3f())


main()




































# import scenic
# from scenic.simulators.webots import WebotsSimulator

# simulator = WebotsSimulator(supervisor)

# path = "/Users/suryasuresh/Desktop/Martial_Arts_Robot/test/Webots/mars/narrow.scenic"
# print(f'Loading Scenic scenario {path}')
# scenario = scenic.scenarioFromFile(path)

# while True:
    # scene, _ = scenario.generate()
    # print('Starting new simulation...')
    # simulator.simulate(scene, verbosity=2)