from controller import Supervisor
import scenic
from scenic.simulators.webots import WebotsSimulator
from math import cos, radians
import time

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

MACH_RADIUS = 18

MIN_DISTANCE = 10.0 

##The minimum permitted distance sensor reading.
SNS_MIN_DISTANCE = MIN_DISTANCE + MACH_RADIUS

##The maximum permitted distance for the machine to be from the opponent it is 
##tracking.
MAX_DISTANCE = 36.0

##The maximum permitted distance sensor reading.
SNS_MAX_DISTANCE = MAX_DISTANCE + MACH_RADIUS

OPP_DISTANCE = MAX_DISTANCE + 18.0

##The opponent distance sensor reading.
SNS_OPP_DISTANCE = OPP_DISTANCE + MACH_RADIUS

supervisor = Supervisor()
simulator = WebotsSimulator(supervisor)

path = "/Users/suryasuresh/Desktop/Martial_Arts_Robot/test/Webots/mars/project.scenic"
print(f'Loading Scenic scenario {path}')

scenario = scenic.scenarioFromFile(path)

def checkRegion(beforeDistances):
    absOppDist = SNS_OPP_DISTANCE / cos(radians(FRONT_ANGLE_MAX - DES_OPP_ANGLE))
    regions = [False] * 5 #red,lightGreen,darkGreen,leftBlue,rightBlue
    for i in range(180, len(beforeDistances)):
        if beforeDistances[i] <= absOppDist:
            # print(i, distances[i])
            if beforeDistances[i] < SNS_MIN_DISTANCE:
                regions[0] = True
                break
            if i <= LEFT_TURN_ANGLE_MAX:
                regions[3] = True
            elif i <= FRONT_ANGLE_MAX:
                absMaxDist = SNS_MAX_DISTANCE / cos(radians(abs(i - DES_OPP_ANGLE)))
                if beforeDistances[i] <= absMaxDist:
                    regions[1] = True
                else:
                    regions[2] = True
            else:
                regions[4] = True
    return regions
    

for i in range(10):
    scene, _ = scenario.generate()
    print('Starting new simulation...')
    
    # time.sleep(1)
    
    robot_node = supervisor.getFromDef("MY_ROBOT")
    
    trans_field = robot_node.getField("translation")
    beforePos = [round(i, 4) for i in trans_field.getSFVec3f()]
    rot_field = robot_node.getField("rotation")
    beforeRot = round(rot_field.getSFRotation()[3],4)
    # print("before:", beforePos, beforeRot)
    
    
    simulator.simulate(scene, verbosity=2)
    print('finished simulating')
    
    beforeDistances = []
    with open('../my_controller_mar/listfile.txt', 'r') as filehandle:
        for line in filehandle:
            # remove linebreak which is the last character of the string
            dist = float(line[:-1])
        
            # add item to the list
            beforeDistances.append(dist)
            
    afterDistances = []
    with open('../my_controller_mar/listfile2.txt', 'r') as filehandle:
        for line in filehandle:
            # remove linebreak which is the last character of the string
            dist = float(line[:-1])
        
            # add item to the list
            afterDistances.append(dist)
    # print('before distances:', beforeDistances[245:296])
    # print()
    # print('afterDistances:',afterDistances[245:296])
    
    afterPos = [round(i, 4) for i in trans_field.getSFVec3f()]
    afterRot = round(rot_field.getSFRotation()[3], 4)
    # print("after:", afterPos, afterRot)
    
    results = ['bRot',beforeRot, 'aRot', afterRot, 'bX', beforePos[0], 'aX', afterPos[0],
                'bY', beforePos[1], 'aY', afterPos[1]]
                
    regions = checkRegion(beforeDistances)
    afterRegions = checkRegion(afterDistances)
    
    results.append('bRegions')
    
    for i in range(len(regions)):
        if regions[i]:
            results.append(i)
    
    results.append('aRegions')
    
    for i in range(len(afterRegions)):
        if afterRegions[i]:
            results.append(i)
    
    results.append('result')
    
    if afterRegions[1] and not afterRegions[0]:
        results.append(True)
        if regions[0]: #red
            if afterPos[1] <= beforePos[1]:
                results[-1] = False
        elif regions[1]:
            if beforePos[0] != afterPos[0] or beforePos[1] != afterPos[1] or beforeRot != afterRot:
                results[-1] = False
        elif regions[2]:
            if afterPos[1] >= beforePos[1]:
                results[-1] = False
        elif regions[3]:
            if beforeRot >= afterRot:
                results[-1] = False
        elif regions[4]:
            if beforeRot <= afterRot:
                results[-1] = False
    else:
        results.append(False)
    print(results)
    with open('results.txt', 'a') as f:
        for item in results:
            f.write("%s " % item)
        f.write("\n")

            
    
    
    
    
                
   
            
            
                
                
            




